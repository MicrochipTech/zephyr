/*
 * Minimal TF-M IPC test for MEC176x bring-up
 * Tests psa_framework_version() without crypto partition
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "psa/client.h"

int main(void)
{
	uint32_t version;

	printk("*** Booting TF-M + Zephyr NS on MEC176x ***\n");

	version = psa_framework_version();
	if (version == PSA_FRAMEWORK_VERSION) {
		printk("SUCCESS: PSA Framework version = 0x%04x\n", version);
	} else {
		printk("FAIL: PSA Framework version mismatch (got 0x%x, expected 0x%x)\n",
		       version, PSA_FRAMEWORK_VERSION);
	}

	printk("*** Test complete ***\n");

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
