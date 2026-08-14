/* Copyright 2026 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

// #include "system.h"

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

static const struct pm_state_info residency_info[] =
	PM_STATE_INFO_LIST_FROM_DT_CPU(DT_NODELABEL(cpu0));

const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
	ARG_UNUSED(cpu);

		/*
		 * Check all states shallowest-first. A lock on any state (e.g.
		 * the UART console wake lock on PM_STATE_SUSPEND_TO_IDLE) means
		 * "stay fully awake" — prevent all sleep states, not just the
		 * locked one. Without this, the deepest-first selection loop
		 * below would pick suspend-to-ram before ever seeing the
		 * suspend-to-idle lock, causing keypresses to be dropped.
		 */
		for (int i = 0; i < ARRAY_SIZE(residency_info); i++) {
			if (pm_policy_state_lock_is_active(
				    residency_info[i].state,
				    PM_ALL_SUBSTATES)) {
				return NULL;
			}
		}

		for (int i = ARRAY_SIZE(residency_info) - 1; i >= 0; i--) {
			if (ticks == K_TICKS_FOREVER ||
			    ticks >= k_us_to_ticks_ceil32(
					    residency_info[i].min_residency_us)) {
				return &residency_info[i];
			}
		}

	return NULL;
}