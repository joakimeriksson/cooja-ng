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
 *   ./test_runner mixed-multinode [opts]   Run mixed MSP430+ARM multi-node test
 *   ./test_runner all [-v]                 Run all (except multinode)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* MSP430 test functions */
extern int run_correctness_tests(int verbose);
extern int run_benchmarks(void);
extern int run_firmware_tests(int verbose);

/* ARM test functions */
extern int run_arm_correctness_tests(int verbose);
extern int run_arm_firmware_tests(int verbose);

/* Mixed-platform test (handles MSP430, ARM, and native nodes) */
extern int run_mixed_multinode_test(int argc, char **argv);

/* Timeline unit tests */
extern int run_timeline_tests(int verbose);

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
        printf("Mixed:        mixed-multinode\n");
        printf("Test:         test <config.json> [-v] [-t ms]\n");
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
        /* Route to mixed-multinode with default MSP430/Sky firmware */
        int extra_argc = argc - 2;
        char **extra_argv = argv + 2;

        /* If no firmware specified, provide default MSP430 firmware.
         * Skip values belonging to -t and -n flags. */
        int has_firmware = 0;
        for (int i = 0; i < extra_argc; i++) {
            if ((strcmp(extra_argv[i], "-t") == 0 ||
                 strcmp(extra_argv[i], "-n") == 0) && i + 1 < extra_argc) {
                i++;  /* skip the value */
            } else if (extra_argv[i][0] != '-') {
                has_firmware = 1; break;
            }
        }
        if (!has_firmware) {
            /* Build new argv with default firmware prepended */
            int new_argc = extra_argc + 1;
            char **new_argv = malloc((new_argc + 1) * sizeof(char *));
            new_argv[0] = (char *)"firmware/sky/nullnet-broadcast.sky";
            for (int i = 0; i < extra_argc; i++)
                new_argv[i + 1] = extra_argv[i];
            new_argv[new_argc] = NULL;
            failures += run_mixed_multinode_test(new_argc, new_argv);
            free(new_argv);
        } else {
            failures += run_mixed_multinode_test(extra_argc, extra_argv);
        }
    }

    /* ARM modes */
    if (strcmp(mode, "arm-correctness") == 0 || strcmp(mode, "all") == 0) {
        failures += run_arm_correctness_tests(verbose);
    }

    if (strcmp(mode, "arm-firmware") == 0) {
        failures += run_arm_firmware_tests(verbose);
    }

    if (strcmp(mode, "arm-multinode") == 0) {
        /* Route to mixed-multinode (ARM firmware auto-detected by extension) */
        failures += run_mixed_multinode_test(argc - 2, argv + 2);
    }

    /* Mixed-platform mode */
    if (strcmp(mode, "mixed-multinode") == 0) {
        failures += run_mixed_multinode_test(argc - 2, argv + 2);
    }

    /* Test scripting mode (alias for mixed-multinode with test assertions) */
    if (strcmp(mode, "test") == 0) {
        failures += run_mixed_multinode_test(argc - 2, argv + 2);
    }

    /* Timeline unit tests */
    if (strcmp(mode, "timeline") == 0 || strcmp(mode, "all") == 0) {
        failures += run_timeline_tests(verbose);
    }

    return failures > 0 ? 1 : 0;
}
