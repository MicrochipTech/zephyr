/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/* The clock the tachometer counts in reading mode 1, datasheet section 29.9.1 */
#define TACH_CLOCK_HZ		100000U
/* pwm_xec_get_cycles_per_sec() always reports the 48 MHz PWM input clock */
#define PWM_XEC_INPUT_HZ	48000000U

/* Readings discarded after a frequency change, then readings compared */
#define TACH_SETTLE_READINGS	2U
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
	/* TACH edges the hardware counts 100 kHz clocks over */
	uint8_t edges;
	/* Half TACH periods that many edges span */
	uint8_t window_half_periods;
	/* TACH periods the emulated fan produces per revolution */
	uint8_t pulses_per_round;
};

#define TACH_INFO_ENTRY(node_id)						\
	{									\
		.dev = DEVICE_DT_GET(node_id),					\
		.edges = DT_PROP(node_id, tach_edges),				\
		.window_half_periods =						\
			TACH_WINDOW_HALF_PERIODS(DT_PROP(node_id, tach_edges)),	\
		.pulses_per_round = DT_PROP(node_id, pulses_per_round),		\
	},

/* Every enabled microchip,xec-tach node, in devicetree order */
static const struct tach_info tachs[] = {
	DT_FOREACH_STATUS_OKAY(microchip_xec_tach, TACH_INFO_ENTRY)
};

BUILD_ASSERT(ARRAY_SIZE(tachs) > 0,
	     "This sample needs at least one enabled microchip,xec-tach node");

static const uint32_t pwm_period_clocks[] = { TACH_PWM_PERIOD_LIST };

static const struct pwm_dt_spec fan = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static unsigned int checks;
static unsigned int failures;

/*
 * The RPM the driver has to report for a period of k clocks. With a window of
 * whp half periods the latched count is whp * k / 2, so
 *
 *   RPM = 60 * 100000 * whp / (2 * pulses_per_round * count)
 *       = 60 * 100000 / (pulses_per_round * k)
 *
 * and whp cancels. The expectation therefore never depends on the
 * edges-to-half-periods mapping under test: a driver that mis-maps tach-edges
 * shows up here as a factor of two or four, not as a wash.
 */
static uint64_t expected_micro_rpm(const struct tach_info *tach, uint32_t period_clocks)
{
	uint64_t numerator = (uint64_t)SEC_PER_MIN * TACH_CLOCK_HZ * USEC_PER_SEC;
	uint64_t denominator = (uint64_t)tach->pulses_per_round * period_clocks;

	/* Round to nearest, matching the driver */
	return (numerator + (denominator / 2U)) / denominator;
}

/*
 * Plus or minus two counts of latch and clock domain skew, plus one micro-RPM
 * of rounding. RPM is inversely proportional to the count, so the count is what
 * sets the resolution: 0.8 % at 2 edges and 200 Hz, 0.006 % at 9 edges and
 * 12.5 Hz. A mis-scaled window would be off by 100 % or 300 %.
 */
static uint64_t micro_rpm_tolerance(const struct tach_info *tach, uint32_t period_clocks,
				    uint64_t expected)
{
	uint32_t count = (uint32_t)tach->window_half_periods * period_clocks / 2U;

	return DIV_ROUND_UP(2U * expected, count) + 1U;
}

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
		uint32_t dhz = TACH_CLOCK_HZ * 10U / k;
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

	printk("instance       edges  half periods  pulses/rev\n");

	for (size_t i = 0; i < ARRAY_SIZE(tachs); i++) {
		const struct tach_info *tach = &tachs[i];

		if (!device_is_ready(tach->dev)) {
			printk("%s: device not ready\n", tach->dev->name);
			return 0;
		}

		printk("%-14s %5u  %12u  %10u\n", tach->dev->name, tach->edges,
		       tach->window_half_periods, tach->pulses_per_round);
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
