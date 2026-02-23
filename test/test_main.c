/*
 * MSP430 C Emulator — Test harness entry point
 *
 * Usage:
 *   ./test_runner correctness [-v]     Run correctness tests
 *   ./test_runner bench                Run performance benchmarks
 *   ./test_runner firmware [-v]        Run firmware tests
 *   ./test_runner multinode [opts]     Run multi-node radio test
 *   ./test_runner all [-v]             Run everything (except multinode)
 */
#include <stdio.h>
#include <string.h>

/* External test functions */
extern int run_correctness_tests(int verbose);
extern int run_benchmarks(void);
extern int run_firmware_tests(int verbose);
extern int run_multinode_test(int argc, char **argv);

int main(int argc, char **argv) {
    int verbose = 0;

    /* Check for -v flag */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        }
    }

    if (argc < 2) {
        printf("Usage: %s <correctness|bench|firmware|multinode|all> [-v]\n", argv[0]);
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

    return failures > 0 ? 1 : 0;
}
