/*
 * MSP430 + ARM C Emulator — Test harness entry point
 *
 * Usage:
 *   ./test_runner correctness [-v]         Run MSP430 correctness tests
 *   ./test_runner bench                    Run MSP430 performance benchmarks
 *   ./test_runner firmware [-v]            Run MSP430 firmware tests
 *   ./test_runner multinode [opts]         Run MSP430 multi-node radio test
 *   ./test_runner arm-correctness [-v]     Run ARM correctness tests
 *   ./test_runner arm-firmware [-v]        Run ARM firmware tests
 *   ./test_runner arm-multinode [opts]     Run ARM multi-node radio test
 *   ./test_runner all [-v]                 Run all (except multinode)
 */
#include <stdio.h>
#include <string.h>

/* MSP430 test functions */
extern int run_correctness_tests(int verbose);
extern int run_benchmarks(void);
extern int run_firmware_tests(int verbose);
extern int run_multinode_test(int argc, char **argv);

/* ARM test functions */
extern int run_arm_correctness_tests(int verbose);
extern int run_arm_firmware_tests(int verbose);
extern int run_arm_multinode_test(int argc, char **argv);

int main(int argc, char **argv) {
    int verbose = 0;

    /* Check for -v flag */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        }
    }

    if (argc < 2) {
        printf("Usage: %s <mode> [-v]\n", argv[0]);
        printf("MSP430 modes: correctness, bench, firmware, multinode\n");
        printf("ARM modes:    arm-correctness, arm-firmware, arm-multinode\n");
        printf("Combined:     all\n");
        return 1;
    }

    const char *mode = argv[1];
    int failures = 0;

    if (strcmp(mode, "correctness") == 0 || strcmp(mode, "all") == 0) {
        failures += run_correctness_tests(verbose);
    }

    if (strcmp(mode, "bench") == 0 || strcmp(mode, "all") == 0) {
        run_benchmarks();
    }

    if (strcmp(mode, "firmware") == 0 || strcmp(mode, "all") == 0) {
        failures += run_firmware_tests(verbose);
    }

    if (strcmp(mode, "multinode") == 0) {
        failures += run_multinode_test(argc - 2, argv + 2);
    }

    /* ARM modes */
    if (strcmp(mode, "arm-correctness") == 0 || strcmp(mode, "all") == 0) {
        failures += run_arm_correctness_tests(verbose);
    }

    if (strcmp(mode, "arm-firmware") == 0) {
        failures += run_arm_firmware_tests(verbose);
    }

    if (strcmp(mode, "arm-multinode") == 0) {
        failures += run_arm_multinode_test(argc - 2, argv + 2);
    }

    return failures > 0 ? 1 : 0;
}
