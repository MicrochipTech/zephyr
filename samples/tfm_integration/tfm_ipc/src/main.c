/*
 * Minimal TF-M IPC test for MEC176x bring-up
 * Tests psa_framework_version() without crypto partition.
 *
 * When the MPU enforcement test partition is built (TFM_PARTITION_MPU_TEST in
 * the TF-M shim config.cmake), also drives the L2 MPU enforcement test cases via
 * the TFM_MPU_TEST_SERVICE and reports PASS/FAIL over the console (RTT).
 * See docs/mpu_enforcement_tests.md.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "psa/client.h"
#include "psa_manifest/sid.h"   /* TFM_MPU_TEST_SERVICE_HANDLE (only when partition built) */

#ifdef TFM_MPU_TEST_SERVICE_HANDLE
/* ---- API IDs - keep in sync with secure_fw/partitions/mpu_test/mpu_test_sp.c ---- */
#define MPU_TEST_API_TT_PROT_DATA   (0x1)  /* TC3 (informational) */
#define MPU_TEST_API_TT_PROT_CODE   (0x2)  /* TC4 (informational) */
#define MPU_TEST_API_TT_AROT_DATA   (0x3)  /* TC5 (informational) */
#define MPU_TEST_API_RW_OWN         (0x4)  /* TC9 positive control (real access) */
#define MPU_TEST_API_READ_PROT_DATA  (0x10) /* TC6 destructive: real unpriv read PSA-RoT data */
#define MPU_TEST_API_WRITE_PROT_DATA (0x11) /* TC7 destructive: real unpriv write PSA-RoT data */
#define MPU_TEST_API_EXEC_PROT_CODE  (0x12) /* TC8 destructive: real unpriv exec PSA-RoT code */

/*
 * Set to one of the destructive API IDs (0x10/0x11/0x12) to run ONE real
 * violation this boot; it is expected to MemManage-fault (the definitive proof
 * the secure MPU is enforcing). 0 = safe build (non-destructive only).
 */
#define MPU_TEST_DESTRUCTIVE   (0)   /* 0 = safe (non-destructive suite only).
                                      * Set to 0x10/0x11/0x12 to run ONE real
                                      * TC6/TC7/TC8 violation (expects MemManage
                                      * fault; read `monitor mdw 0x2000FE00 8`). */

#define RES_DENIED    (0x0u)
#define RES_ALLOWED   (0x1u)
#define RES_RW_OK     (0xA5u)

static uint32_t mpu_test_call(uint32_t api_id)
{
	uint32_t result = 0xFFFFFFFFu;
	psa_outvec out_vec[] = { { &result, sizeof(result) } };
	psa_status_t st = psa_call(TFM_MPU_TEST_SERVICE_HANDLE, (int32_t)api_id,
				   NULL, 0, out_vec, 1);

	if (st != PSA_SUCCESS) {
		printk("  psa_call(api=0x%x) failed: %d\n", api_id, st);
		return 0xFFFFFFFFu;
	}
	return result;
}

static void run_mpu_enforcement_tests(void)
{
	uint32_t r;

	printk("--- MPU enforcement tests (isolation L2) ---\n");

	/* TC3-TC5: TT queries. INFORMATIONAL ONLY - cmse_check_address_range/TT is
	 * unreliable as a positive check from an ALREADY-unprivileged partition
	 * (it returns NULL even for genuinely-accessible unpriv memory). The
	 * authoritative proof is the real-access pair TC9 (allow) + TC6 (fault). */
	r = mpu_test_call(MPU_TEST_API_TT_PROT_DATA);
	printk("TC3 TT PSA-RoT data (unpriv): r=0x%x [info: TT-from-unpriv unreliable]\n", r);
	r = mpu_test_call(MPU_TEST_API_TT_PROT_CODE);
	printk("TC4 TT PSA-RoT code (unpriv): r=0x%x [info]\n", r);
	r = mpu_test_call(MPU_TEST_API_TT_AROT_DATA);
	printk("TC5 TT App-RoT data (unpriv): r=0x%x [info]\n", r);

	/* TC9 positive control - REAL access to own App-RoT data (authoritative). */
	r = mpu_test_call(MPU_TEST_API_RW_OWN);
	printk("TC9 own App-RoT data RW:      %s (r=0x%x, expect 0xA5) [REAL, authoritative]\n",
	       (r == RES_RW_OK) ? "PASS" : "FAIL", r);

#if MPU_TEST_DESTRUCTIVE
	/* TC6/7/8: the definitive negative test. ONE real illegal access to PSA-RoT
	 * from this unprivileged partition -> expected MemManage fault + halt.
	 * After the hang: read `monitor mdw 0x2000FE00 8`
	 *   [0]=CFSR (MMFSR: bit1 DACCVIOL / bit0 IACCVIOL, bit7 MMARVALID)
	 *   [5]=MMFAR (== PSA-RoT target)  [7]=0xFA0117ED marker. */
	printk("TC6/7/8 DESTRUCTIVE: unpriv illegal access to PSA-RoT (api=0x%x)\n",
	       MPU_TEST_DESTRUCTIVE);
	printk("  expect: MemManage fault + halt now. Read: monitor mdw 0x2000FE00 8\n");
	r = mpu_test_call(MPU_TEST_DESTRUCTIVE);
	printk("  UNEXPECTED: returned r=0x%x without faulting -> MPU NOT enforcing!\n", r);
#endif

	printk("--- MPU enforcement tests done ---\n");
}
#endif /* TFM_MPU_TEST_SERVICE_HANDLE */

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

#ifdef TFM_MPU_TEST_SERVICE_HANDLE
	run_mpu_enforcement_tests();
#endif

	printk("*** Test complete ***\n");

	while (1) {
		k_sleep(K_SECONDS(1));
#ifdef TFM_MPU_TEST_SERVICE_HANDLE
		/* Periodic re-entry into the secure test service so a debugger can
		 * catch a SECURE halt (bp on tfm_mpu_test_service_sfn) to read the
		 * banked MPU_CTRL_S / dump regions (TC1/TC2). Non-destructive. */
		(void)mpu_test_call(MPU_TEST_API_TT_AROT_DATA);
#endif
	}

	return 0;
}
