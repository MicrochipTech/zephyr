/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/*
 * Nominal frequency of the divided clock the tachometer counts in reading mode
 * 1, datasheet section 29.9.1, and of the PWM low clock the sweep programs its
 * period in. What the driver converts with is the clock-frequency property of
 * each instance, which a board that has measured its own clock may set a little
 * either side of this, so every expectation below comes from that property and
 * only the programmed period is expressed in nominal clocks.
 */
#define TACH_NOMINAL_CLOCK_HZ	100000U

/* Readings discarded before the first one that is used */
#define TACH_SETTLE_READINGS	2U

/*
 * Half TACH periods the block measures over for each TACH_EDGES setting. The
 * edge count and the periods spanned are different numbers: a TACH period is
 * three edges and the settings share endpoints, so 2, 3, 5 and 9 edges span
 * 1, 2, 4 and 8 half periods.
 */
#define TACH_WINDOW_HALF_PERIODS(edges)						\
	(((edges) == 2) ? 1U : ((edges) == 3) ? 2U : ((edges) == 5) ? 4U : 8U)

#define TACH_ASSERT_EDGES(node_id)						\
	BUILD_ASSERT((DT_PROP(node_id, tach_edges) == 2) ||			\
		     (DT_PROP(node_id, tach_edges) == 3) ||			\
		     (DT_PROP(node_id, tach_edges) == 5) ||			\
		     (DT_PROP(node_id, tach_edges) == 9),			\
		     "tach-edges must be 2, 3, 5 or 9");

DT_FOREACH_STATUS_OKAY(microchip_xec_tach, TACH_ASSERT_EDGES)

struct tach_info {
	const struct device *dev;
	/* TACH edges the hardware counts clocks over */
	uint8_t edges;
	/* Half TACH periods that many edges span */
	uint8_t window_half_periods;
	/* TACH periods the emulated fan produces per revolution */
	uint8_t pulses_per_round;
	/* Frequency in Hz of the clock the driver converts with */
	uint32_t clock_hz;
};

#define TACH_INFO_ENTRY(node_id)						\
	{									\
		.dev = DEVICE_DT_GET(node_id),					\
		.edges = DT_PROP(node_id, tach_edges),				\
		.window_half_periods =						\
			TACH_WINDOW_HALF_PERIODS(DT_PROP(node_id, tach_edges)),	\
		.pulses_per_round = DT_PROP(node_id, pulses_per_round),		\
		.clock_hz = DT_PROP(node_id, clock_frequency),			\
	},

/* Every enabled microchip,xec-tach node, in devicetree order */
static const struct tach_info tachs[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_tach, TACH_INFO_ENTRY)
};

BUILD_ASSERT(ARRAY_SIZE(tachs) > 0,
	     "This sample needs at least one enabled microchip,xec-tach node");

static int tach_read_micro_rpm(const struct tach_info *tach, int64_t *micro_rpm)
{
	struct sensor_value rpm;
	int ret;

	ret = sensor_sample_fetch_chan(tach->dev, SENSOR_CHAN_RPM);
	if (ret != 0) {
		return ret;
	}

	ret = sensor_channel_get(tach->dev, SENSOR_CHAN_RPM, &rpm);
	if (ret != 0) {
		return ret;
	}

	*micro_rpm = ((int64_t)rpm.val1 * (int64_t)USEC_PER_SEC) + (int64_t)rpm.val2;

	return 0;
}

static bool tachs_are_ready(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		if (!device_is_ready(tachs[i].dev)) {
			printk("%s: device not ready\n", tachs[i].dev->name);
			return false;
		}
	}

	return true;
}

#if defined(CONFIG_APP_RAW_COUNT_MODE)

/* Latched counts averaged per instance */
#define RAW_READINGS	((unsigned int)CONFIG_APP_RAW_COUNT_READINGS)

/* Averages and offsets are carried in thousandths of a count */
#define MILLI		1000

/* Estimates must agree to a quarter of a count for the result to be reported */
#define RAW_AGREEMENT	(MILLI / 4)

/*
 * Numerator of the driver's conversion before the window width is applied,
 * 60 * clock_frequency * 1000000 / 2. tach_xec_channel_get() reports
 *
 *   micro_rpm = round(TACH_MICRO_RPM_NUM(hz) * whp / (pulses_per_round * count))
 *
 * so the count the block latched is recoverable from the value it returned. The
 * frequency is whatever devicetree gave the driver, so the recovery stays exact
 * on a board that has calibrated it, and it cancels out of the pair estimate
 * below in any case.
 */
#define TACH_MICRO_RPM_NUM(hz)	((uint64_t)(hz) * SEC_PER_MIN * USEC_PER_SEC / 2U)

struct raw_stats {
	/* Sum of the latched counts and their extremes */
	uint64_t sum;
	uint32_t min;
	uint32_t max;
	/* Mean latched count, in thousandths of a count */
	uint32_t mean_milli;
};

static struct raw_stats stats[ARRAY_SIZE(tachs)];

/*
 * Recover the count the driver converted from the RPM it reported, which is the
 * latched count plus the offset the driver corrects for. Rounding to the
 * nearest micro-RPM displaces the result by at most count / (2 * micro_rpm).
 * The count cannot exceed 0xfffe, and the micro-RPM that corresponds to such a
 * count cannot be smaller than about 1.1e7, so the displacement stays below
 * 0.003 counts and the value below is the latched count exactly.
 */
static uint32_t raw_count(const struct tach_info *tach, int64_t micro_rpm)
{
	uint64_t numerator = TACH_MICRO_RPM_NUM(tach->clock_hz) * tach->window_half_periods;
	uint64_t denominator = (uint64_t)tach->pulses_per_round * (uint64_t)micro_rpm;

	/* Round to nearest, matching the driver */
	return (uint32_t)((numerator + (denominator / 2U)) / denominator);
}

static void print_signed_milli(int64_t milli)
{
	uint32_t magnitude = (uint32_t)((milli < 0) ? -milli : milli);

	printk("%s%u.%03u", (milli < 0) ? "-" : "", magnitude / MILLI, magnitude % MILLI);
}

/* Signed divide, rounding to nearest rather than toward zero */
static int64_t div_round(int64_t numerator, int64_t denominator)
{
	if (numerator < 0) {
		return -(((-numerator) + (denominator / 2)) / denominator);
	}

	return (numerator + (denominator / 2)) / denominator;
}

static int measure_one(const struct tach_info *tach, struct raw_stats *st)
{
	st->sum = 0U;
	st->min = UINT32_MAX;
	st->max = 0U;

	for (unsigned int i = 0; i < TACH_SETTLE_READINGS; i++) {
		int64_t discard;

		/* The block may already have a count latched from before boot */
		(void)tach_read_micro_rpm(tach, &discard);
	}

	for (unsigned int i = 0; i < RAW_READINGS; i++) {
		int64_t micro_rpm;
		uint32_t count;
		int ret;

		ret = tach_read_micro_rpm(tach, &micro_rpm);
		if (ret != 0) {
			printk("%s: reading %u failed (%d)\n", tach->dev->name, i, ret);
			return ret;
		}

		/*
		 * The driver reports zero for a saturated count, which here
		 * means the input is not reaching this instance at all.
		 */
		if (micro_rpm <= 0) {
			printk("%s: no signal on reading %u\n", tach->dev->name, i);
			return -ENODATA;
		}

		count = raw_count(tach, micro_rpm);
		st->sum += count;
		st->min = MIN(st->min, count);
		st->max = MAX(st->max, count);
	}

	st->mean_milli = (uint32_t)(((st->sum * MILLI) + (RAW_READINGS / 2U)) / RAW_READINGS);

	return 0;
}

/*
 * The measurement window of an instance spans whp half periods of the input, so
 * for an input period of M clocks of 100 kHz the block latches whp * M / 2 less
 * the offset the driver corrects for. This mode measures the residual offset c
 * in
 *
 *   count as converted by the driver = whp * M / 2 + c
 *
 * from an input that is not locked to the SoC clock, so a correct driver reports
 * 0 here. The count is recovered from the driver's own output, which means what
 * is measured is the whole path: an offset the block applies and the driver does
 * not correct shows up, and so does the reverse.
 *
 * Two instances whose window widths are in a 1:2 ratio give
 *
 *   c = 2 * mean(count for whp) - mean(count for 2 * whp)
 *
 * in which M cancels, so the accuracy of the generator does not enter the
 * result and its frequency does not even have to be known. What the generator
 * must do is drift: the mean of a truncating counter is only the true window
 * width if the phase between the input and the 100 kHz clock is spread over the
 * readings, which is why the frequency should not be a round number of clocks.
 */
static void report(void)
{
	int64_t c_sum = 0;
	int64_t c_min = 0;
	int64_t c_max = 0;
	int64_t c_milli;
	int64_t c_whole;
	int64_t residual;
	unsigned int pairs = 0U;
	bool dithered = false;

	printk("\n--- latched counts, %u readings per instance ---\n", RAW_READINGS);
	printk("instance       edges  half periods  readings    min    max  mean count\n");

	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		printk("%-14s %5u  %12u  %8u  %5u  %5u  %6u.%03u\n", tachs[i].dev->name,
		       tachs[i].edges, tachs[i].window_half_periods, RAW_READINGS,
		       stats[i].min, stats[i].max, stats[i].mean_milli / MILLI,
		       stats[i].mean_milli % MILLI);

		if (stats[i].min != stats[i].max) {
			dithered = true;
		}
	}

	printk("\n--- offset implied by each pair of windows in a 1:2 ratio ---\n");
	printk("instance a     instance b      whp a  whp b   offset c\n");

	for (size_t a = 0; a < ARRAY_SIZE(tachs); a++) {
		for (size_t b = 0; b < ARRAY_SIZE(tachs); b++) {
			int64_t offset;

			if (tachs[b].window_half_periods !=
			    (2U * tachs[a].window_half_periods)) {
				continue;
			}

			offset = (2 * (int64_t)stats[a].mean_milli) -
				 (int64_t)stats[b].mean_milli;

			printk("%-14s %-14s %6u %6u   ", tachs[a].dev->name,
			       tachs[b].dev->name, tachs[a].window_half_periods,
			       tachs[b].window_half_periods);
			print_signed_milli(offset);
			printk("\n");

			if (pairs == 0U) {
				c_min = offset;
				c_max = offset;
			}

			c_min = MIN(c_min, offset);
			c_max = MAX(c_max, offset);
			c_sum += offset;
			pairs++;
		}
	}

	if (pairs == 0U) {
		printk("(none)\n\n"
		       "No two enabled instances have measurement windows in a 1:2 ratio, so\n"
		       "the offset cannot be separated from the input period. Give two\n"
		       "instances adjacent tach-edges values out of 2, 3, 5 and 9.\n");
		printk("TACH XEC latched count offset: not measurable\n");
		return;
	}

	c_milli = div_round(c_sum, (int64_t)pairs);
	c_whole = div_round(c_milli, MILLI);
	residual = c_milli - (c_whole * MILLI);

	printk("\n--- input implied by each instance at that offset ---\n");
	printk("instance      period clocks  clock Hz  frequency Hz\n");

	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		int64_t whp = (int64_t)tachs[i].window_half_periods;
		int64_t period;
		uint64_t period_milli;
		uint64_t freq_milli;

		/* M = 2 * (count - c) / whp, and the clock over M is the frequency */
		period = div_round(2 * ((int64_t)stats[i].mean_milli - c_milli), whp);
		if (period <= 0) {
			continue;
		}

		period_milli = (uint64_t)period;

		freq_milli = (((uint64_t)tachs[i].clock_hz * MILLI * MILLI) +
			      (period_milli / 2U)) / period_milli;

		printk("%-14s %8u.%03u  %8u  %8u.%03u\n", tachs[i].dev->name,
		       (uint32_t)(period_milli / MILLI), (uint32_t)(period_milli % MILLI),
		       tachs[i].clock_hz, (uint32_t)(freq_milli / MILLI),
		       (uint32_t)(freq_milli % MILLI));
	}

	printk("\noffset c = ");
	print_signed_milli(c_milli);
	printk(" counts from %u pair%s, spread ", pairs, (pairs == 1U) ? "" : "s");
	print_signed_milli(c_max - c_min);
	printk("\n\n");

	if (!dithered) {
		printk("Every reading of every instance latched the same count, so the input\n"
		       "is phase locked to the tachometer clock and each window ends on the\n"
		       "same counter clock edge. An offset measured this way cannot be told\n"
		       "apart from that coincidence: drive the inputs from a free running\n"
		       "generator, at a frequency that is not a round number of 100 kHz\n"
		       "clocks.\n");
		printk("TACH XEC latched count offset: inconclusive\n");
		return;
	}

	if (((c_max - c_min) > RAW_AGREEMENT) || (residual > RAW_AGREEMENT) ||
	    (residual < -RAW_AGREEMENT)) {
		printk("The pair estimates do not agree on a whole number of counts. Either\n"
		       "the input frequency moved during the run, the instances are not all\n"
		       "driven from the same source, or more readings are needed.\n");
		printk("TACH XEC latched count offset: inconclusive\n");
		return;
	}

	if (c_whole == 0) {
		printk("The count the driver converts is the number of 100 kHz clocks the\n"
		       "measurement window took, so the conversion is correct as it stands.\n");
	} else if (c_whole == -1) {
		printk("The count the driver converts is one clock short of the window it\n"
		       "measured, so the reported RPM is high by one part in that count: 0.4 %%\n"
		       "at 2 edges and 200 Hz. The block latches one less than the window\n"
		       "width and the driver is expected to add it back, so seeing this means\n"
		       "the correction in tach_xec_channel_get() has been lost.\n");
	} else {
		printk("That offset is neither 0 nor -1, which no reading of the datasheet\n"
		       "predicts. Check the wiring and the generator before believing it.\n");
	}

	printk("TACH XEC latched count offset: ");
	print_signed_milli(c_milli);
	printk(" counts\n");
}

int main(void)
{
	printk("Microchip XEC tachometer latched count measurement\n\n");
	printk("Remove the PWM0 fly-wire, then drive GPIO050, GPIO051, GPIO052 and\n");
	printk("GPIO033 from one free running generator sharing the board ground: a\n");
	printk("0 to 3.3 V square wave, 50 %% duty, at a frequency that is not a round\n");
	printk("number of 100 kHz clocks, such as 120 Hz. Do not exceed 3.3 V - two\n");
	printk("of those pins have no over-voltage protection.\n\n");

	if (!tachs_are_ready()) {
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		if (measure_one(&tachs[i], &stats[i]) != 0) {
			printk("TACH XEC latched count offset: not measured\n");
			return 0;
		}
	}

	report();

	return 0;
}

#else /* CONFIG_APP_RAW_COUNT_MODE */

/* pwm_xec_get_cycles_per_sec() always reports the 48 MHz PWM input clock */
#define PWM_XEC_INPUT_HZ	48000000U

/* Readings compared after the settling readings at each frequency */
#define TACH_TEST_READINGS	3U
/* Time for the PWM output to restart before the first window is armed */
#define TACH_PWM_SETTLE		K_MSEC(20)

/*
 * Emulated fan periods, in 100 kHz clocks: 200, 125, 100, 62.5, 25 and 12.5 Hz,
 * which is 6000 down to 375 RPM at two TACH periods per revolution.
 *
 * A period of k clocks and a window of whp half periods latch a count of
 * whp * k / 2. The widest window is 8 half periods, so the largest count here
 * is 32000 (320 ms) - clear of both the 0xffff saturation at 655 ms and the
 * driver's 700 ms fetch timeout. The smallest is 250 (2.5 ms), which holds
 * plus or minus one count of quantisation to 0.4 %. The driver enables
 * FILTER_ENABLE, which discards pulses narrower than two 100 kHz periods, so
 * the shortest half period here is still two orders of magnitude clear of it.
 *
 * 1600 and 8000 make the expected RPM fractional at 4 pulses per revolution
 * (937.5 and 187.5), which is the only thing here that exercises val2.
 */
#define TACH_PWM_PERIOD_LIST	500, 800, 1000, 1600, 4000, 8000

/*
 * Requesting these cycle counts drives pwm_xec_set_cycles() to an exactly
 * symmetric period of k clocks of the 100 kHz PWM clock. This is the one place
 * the sample depends on driver internals, so the derivation is spelled out.
 *
 * pwm_xec_set_cycles() takes on = pulse and off = period - pulse, both in
 * 48 MHz cycles, so the values below give on = off = 240k - 1 and
 *
 *   target_freq = 48000000 * 10 / ((on + 1) + (off + 1)) = 1000000 / k
 *
 * in units of 0.1 Hz, and a duty cycle of exactly 50.000 %. That frequency is
 * below XEC_PWM_MAX_LOW_CLK_FREQ, so the 100 kHz clock is a candidate. At
 * divider 0 that clock gives on_off = 100000 * 10 / target_freq = k and
 * COUNT_ON = COUNT_OFF = k / 2 - 1, reproducing target_freq with no error at
 * all. xec_compare_div_on_off() and xec_compare_params() both compare with a
 * strict less-than, so a zero-error candidate is never displaced and the low
 * clock wins the tie against the 48 MHz clock. The result is k / 2 clocks high
 * and k / 2 clocks low.
 *
 * That holds only when target_freq is a whole number of 0.1 Hz units and k / 2
 * is a whole number of clocks, hence the assertions on the period list.
 */
#define XEC_PWM_PERIOD_CYCLES(k)	(480U * (k) - 2U)
#define XEC_PWM_PULSE_CYCLES(k)		(240U * (k) - 1U)

#define ASSERT_PERIOD_CLOCKS(k)							\
	BUILD_ASSERT((((k) % 2U) == 0U) && ((1000000U % (k)) == 0U) &&		\
		     ((4U * (k)) < 0xffffU),					\
		     "PWM period must be even, divide 1000000, and keep the "	\
		     "widest measurement window below counter saturation")

FOR_EACH(ASSERT_PERIOD_CLOCKS, (;), TACH_PWM_PERIOD_LIST);

static const uint32_t pwm_period_clocks[] = { TACH_PWM_PERIOD_LIST };

static const struct pwm_dt_spec fan = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static unsigned int checks;
static unsigned int failures;

/*
 * The RPM the driver has to report for a period of k clocks. With a window of
 * whp half periods the latched count is whp * k / 2, so
 *
 *   RPM = 60 * clock_frequency * whp / (2 * pulses_per_round * count)
 *       = 60 * clock_frequency / (pulses_per_round * k)
 *
 * and whp cancels. The expectation therefore never depends on the
 * edges-to-half-periods mapping under test: a driver that mis-maps tach-edges
 * shows up here as a factor of two or four, not as a wash.
 *
 * The frequency here is the clock-frequency property, not the nominal 100 kHz,
 * because the emulated period k is a number of clocks rather than a time: the
 * PWM divides the same clock the tachometer counts, so a clock that is off by a
 * percent stretches the fan period and the measurement window alike and the
 * comparison is blind to it either way. Calibrating the property scales what the
 * driver reports, and the expectation has to scale with it or a calibrated board
 * would fail every row by the calibration factor.
 */
static uint64_t expected_micro_rpm(const struct tach_info *tach, uint32_t period_clocks)
{
	uint64_t numerator = (uint64_t)SEC_PER_MIN * tach->clock_hz * USEC_PER_SEC;
	uint64_t denominator = (uint64_t)tach->pulses_per_round * period_clocks;

	/* Round to nearest, matching the driver */
	return (numerator + (denominator / 2U)) / denominator;
}

/*
 * Plus or minus one count of latch and clock domain skew, plus one micro-RPM of
 * rounding. RPM is inversely proportional to the count, so the count is what
 * sets the resolution: 0.4 % at 2 edges and 200 Hz, 0.003 % at 9 edges and
 * 12.5 Hz. A mis-scaled window would be off by 100 % or 300 %.
 *
 * This was two counts while the block's one count latch offset was unexplained.
 * CONFIG_APP_RAW_COUNT_MODE measured it, the driver corrects for it, and the
 * error columns below should now read a few ppm rather than most of a count.
 */
static uint64_t micro_rpm_tolerance(const struct tach_info *tach, uint32_t period_clocks,
				    uint64_t expected)
{
	uint32_t count = (uint32_t)tach->window_half_periods * period_clocks / 2U;

	return DIV_ROUND_UP(expected, count) + 1U;
}

static void print_micro_rpm(int64_t micro_rpm)
{
	printk("%7u.%06u", (uint32_t)(micro_rpm / USEC_PER_SEC),
	       (uint32_t)(micro_rpm % USEC_PER_SEC));
}

static void check_one(const struct tach_info *tach, uint32_t period_clocks)
{
	uint64_t expected = expected_micro_rpm(tach, period_clocks);
	uint64_t tolerance = micro_rpm_tolerance(tach, period_clocks, expected);

	for (unsigned int i = 0; i < TACH_SETTLE_READINGS; i++) {
		int64_t discard;

		/*
		 * The window straddling the frequency change latches a count
		 * that belongs to neither frequency.
		 */
		(void)tach_read_micro_rpm(tach, &discard);
	}

	for (unsigned int i = 0; i < TACH_TEST_READINGS; i++) {
		int64_t measured;
		int64_t error;
		int32_t error_ppm;
		bool pass;
		int ret;

		checks++;

		ret = tach_read_micro_rpm(tach, &measured);
		if (ret != 0) {
			failures++;
			printk("%-14s ", tach->dev->name);
			print_micro_rpm((int64_t)expected);
			printk("         read failed (%d)  FAIL\n", ret);
			continue;
		}

		error = measured - (int64_t)expected;
		error_ppm = (int32_t)((error * (int64_t)USEC_PER_SEC) / (int64_t)expected);
		pass = ((error < 0) ? -error : error) <= (int64_t)tolerance;
		if (!pass) {
			failures++;
		}

		printk("%-14s ", tach->dev->name);
		print_micro_rpm((int64_t)expected);
		printk(" ");
		print_micro_rpm(measured);
		printk(" %8dppm  %s\n", error_ppm, pass ? "PASS" : "FAIL");
	}
}

/*
 * With no edges arriving the counter saturates and the block latches 0xffff,
 * which the driver reports as a stopped fan. This is the only coverage of that
 * path, and of the 655 ms saturation fitting inside the 700 ms fetch timeout.
 */
static void check_stopped(void)
{
	printk("\n--- PWM output disabled: every instance must report 0 RPM ---\n");

	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		const struct tach_info *tach = &tachs[i];
		int64_t measured;
		int ret;

		checks++;

		ret = tach_read_micro_rpm(tach, &measured);
		if (ret != 0) {
			failures++;
			printk("%-14s read failed (%d)  FAIL\n", tach->dev->name, ret);
			continue;
		}

		if (measured != 0) {
			failures++;
		}

		printk("%-14s ", tach->dev->name);
		print_micro_rpm(measured);
		printk(" RPM  %s\n", (measured == 0) ? "PASS" : "FAIL");
	}
}

static int sweep(void)
{
	for (size_t p = 0; p < ARRAY_SIZE(pwm_period_clocks); p++) {
		uint32_t k = pwm_period_clocks[p];
		/* Emulated fan frequency in units of 0.1 Hz */
		uint32_t dhz = TACH_NOMINAL_CLOCK_HZ * 10U / k;
		int ret;

		ret = pwm_set_cycles(fan.dev, fan.channel, XEC_PWM_PERIOD_CYCLES(k),
				     XEC_PWM_PULSE_CYCLES(k), fan.flags);
		if (ret != 0) {
			printk("pwm_set_cycles() failed for %u clocks (%d)\n", k, ret);
			return ret;
		}

		k_sleep(TACH_PWM_SETTLE);

		printk("\n--- %u.%u Hz, period %u clocks of 100 kHz ---\n",
		       dhz / 10U, dhz % 10U, k);
		printk("instance         expected RPM   measured RPM       error  result\n");

		for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
			check_one(&tachs[i], k);
		}
	}

	return 0;
}

int main(void)
{
	uint64_t pwm_cycles;
	int ret;

	printk("Microchip XEC tachometer PWM loopback self test\n\n");
	printk("Fly-wire %s (PWM0, GPIO053) to the TACH inputs GPIO050, GPIO051,\n",
	       fan.dev->name);
	printk("GPIO052 and GPIO033, sharing the board ground.\n\n");

	if (!pwm_is_ready_dt(&fan)) {
		printk("%s: device not ready\n", fan.dev->name);
		return 0;
	}

	/*
	 * XEC_PWM_PERIOD_CYCLES() is expressed in cycles of this clock, so a
	 * different rate would silently change every programmed frequency.
	 */
	ret = pwm_get_cycles_per_sec(fan.dev, fan.channel, &pwm_cycles);
	if (ret != 0) {
		printk("%s: pwm_get_cycles_per_sec() failed (%d)\n", fan.dev->name, ret);
		return 0;
	}

	if (pwm_cycles != PWM_XEC_INPUT_HZ) {
		printk("%s: expected a %u Hz PWM clock, got %u Hz\n", fan.dev->name,
		       PWM_XEC_INPUT_HZ, (uint32_t)pwm_cycles);
		return 0;
	}

	printk("instance       edges  half periods  pulses/rev  clock Hz\n");

	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		const struct tach_info *tach = &tachs[i];

		printk("%-14s %5u  %12u  %10u  %8u\n", tach->dev->name, tach->edges,
		       tach->window_half_periods, tach->pulses_per_round, tach->clock_hz);
	}

	if (!tachs_are_ready()) {
		return 0;
	}

	if (sweep() != 0) {
		printk("TACH XEC PWM loopback: FAIL\n");
		return 0;
	}

	ret = pwm_set_cycles(fan.dev, fan.channel, 0U, 0U, fan.flags);
	if (ret != 0) {
		printk("%s: could not stop the output (%d)\n", fan.dev->name, ret);
		printk("TACH XEC PWM loopback: FAIL\n");
		return 0;
	}

	check_stopped();

	printk("\n%u of %u checks passed\n", checks - failures, checks);

	if (failures == 0U) {
		printk("TACH XEC PWM loopback: PASS\n");
	} else {
		printk("TACH XEC PWM loopback: FAIL\n");
	}

	return 0;
}

#endif /* CONFIG_APP_RAW_COUNT_MODE */
