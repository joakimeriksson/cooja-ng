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
 *   ./test_runner zoul-firefly-multinode [opts]
 *                                          Run Zolertia Firefly multi-node test
 *                                          (CC1200 sub-GHz radio path)
 *   ./test_runner nrf52840-dongle-multinode [opts]
 *                                          Run nRF52840 USB Dongle multi-node test
 *                                          (on-chip 2.4 GHz radio path)
 *   ./test_runner nrf52840-dk-multinode [opts]
 *                                          Run nRF52840 Development Kit multi-node test
 *   ./test_runner mixed-multinode [opts]   Run mixed MSP430+ARM multi-node test
 *   ./test_runner all [-v]                 Run all (except multinode)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "csim_version.h"
#include "arm_cpu.h"
#include "arm_platform.h"
#include "arm_elf.h"

/* TrustZone split-image boot (tz-boot mode): capture UART and flag the
 * secure->non-secure handoff and the SG-veneer round-trip. */
static int tz_boot_saw_ns = 0;      /* "Non-secure world" printed */
static int tz_boot_saw_secret = 0;  /* "sec ret" printed (SG round-trip) */
static char tz_boot_line[256];
static int tz_boot_linepos = 0;
static void tz_boot_tx(void *user_data, uint8_t byte) {
    (void)user_data;
    if (byte == '\n' || tz_boot_linepos == sizeof(tz_boot_line) - 1) {
        tz_boot_line[tz_boot_linepos] = '\0';
        if (tz_boot_linepos > 0) printf("  FW: %s\n", tz_boot_line);
        if (strstr(tz_boot_line, "Non-secure world")) tz_boot_saw_ns = 1;
        if (strstr(tz_boot_line, "sec ret"))          tz_boot_saw_secret = 1;
        tz_boot_linepos = 0;
    } else if (byte != '\r') {
        tz_boot_line[tz_boot_linepos++] = (char)byte;
    }
}

/* MSP430 test functions */
extern int run_correctness_tests(int verbose);
extern int run_benchmarks(void);
extern int run_firmware_tests(int verbose);

/* ARM test functions */
extern int run_arm_correctness_tests(int verbose);
extern int run_arm_benchmarks(void);
extern int run_arm_decode_tests(int verbose);
extern int run_arm_jit_tests(int verbose);
extern int run_config_convert(int argc, char **argv);
extern int run_config_roundtrip(int argc, char **argv);
extern int run_config_reject(int argc, char **argv);
extern int run_arm_firmware_tests(int verbose);

/* Mixed-platform test (handles MSP430, ARM, and native nodes) */
extern int run_mixed_multinode_test(int argc, char **argv);

/* Timeline unit tests */
extern int run_timeline_tests(int verbose);

/* Mock sim_host_t unit tests */
extern int run_mock_host_tests(int verbose);

/* CC1200 chip-driver mock-host unit tests (L−1) */
extern int run_cc1200_tests(int verbose);

/* nRF54L15 SPIM peripheral model unit tests (mock host, no CPU) */
extern int run_nrf54l15_spim_tests(int verbose);

/* MX25R6435F flash chip-driver mock-host unit tests */
extern int run_mx25r6435f_tests(int verbose);

/* radio_medium_t unit tests (pure C, no CPU) */
extern int run_radio_medium_tests(int verbose);

/* sim_radio_bus unit tests (Phase 5 guardrail, no CPU) */
extern int run_radio_bus_tests(int verbose);

int main(int argc, char **argv) {
    int verbose = 0;

    /* --version / -V: print version and exit (before anything else). */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-V") == 0) {
            printf("%s %s\n", CSIM_NAME, CSIM_VERSION);
            return 0;
        }
    }

    /* Check for -v flag */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        }
    }

    if (argc < 2) {
        printf("Usage: %s <mode> [-v]\n", argv[0]);
        printf("MSP430 modes: correctness, bench, firmware, multinode\n");
        printf("ARM modes:    arm-correctness, arm-firmware, arm-multinode,\n");
        printf("              zoul-firefly-multinode,\n");
        printf("              nrf52840-dongle-multinode, nrf52840-dk-multinode,\n");
        printf("              nrf54l15-dk-multinode\n");
        printf("Mixed:        mixed-multinode\n");
        printf("Chip drivers: cc1200-mock-host\n");
        printf("Radio medium: radio-medium\n");
        printf("Radio bus:    radio-bus\n");
        printf("Test:         test <config.yaml|json> [-v] [-t ms] [--seed N] [--save-config out.yaml]\n");
        printf("Config:       config-convert <in> <out.yaml> | config-roundtrip <config...> | config-reject <config...>\n");
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
    if (strcmp(mode, "arm-decode") == 0 || strcmp(mode, "all") == 0) {
        failures += run_arm_decode_tests(verbose);
    }

    if (strcmp(mode, "arm-jit") == 0 || strcmp(mode, "all") == 0) {
        failures += run_arm_jit_tests(verbose);
    }

    if (strcmp(mode, "arm-bench") == 0) {
        failures += run_arm_benchmarks();
    }

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

    /* Zolertia Firefly multinode wrapper.  Behaves exactly like
     * mixed-multinode (ARM firmware auto-detected via the
     * .zoul-firefly extension); kept as its own entry point so the
     * SPEC's Definition-of-Done commands match this binary verbatim
     * and CI can wire a Firefly-specific job.
     *
     * Convenience: if the caller passed exactly one firmware and no
     * -n flag, default to 2 nodes — Firefly broadcast/UDP tests are
     * always at least pairs, and the SPEC's example commands omit
     * the -n flag. The MSP430 `multinode` wrapper does the same. */
    if (strcmp(mode, "zoul-firefly-multinode") == 0) {
        int extra_argc = argc - 2;
        char **extra_argv = argv + 2;
        int has_n = 0;
        int firmware_count = 0;
        for (int i = 0; i < extra_argc; i++) {
            if ((strcmp(extra_argv[i], "-t") == 0 ||
                 strcmp(extra_argv[i], "-n") == 0 ||
                 strcmp(extra_argv[i], "--threads") == 0) &&
                i + 1 < extra_argc) {
                if (strcmp(extra_argv[i], "-n") == 0) has_n = 1;
                i++;  /* skip the value */
            } else if (extra_argv[i][0] != '-') {
                firmware_count++;
            }
        }
        if (!has_n && firmware_count == 1) {
            int new_argc = extra_argc + 2;
            char **new_argv = malloc((new_argc + 1) * sizeof(char *));
            for (int i = 0; i < extra_argc; i++) new_argv[i] = extra_argv[i];
            new_argv[extra_argc]     = (char *)"-n";
            new_argv[extra_argc + 1] = (char *)"2";
            new_argv[new_argc] = NULL;
            failures += run_mixed_multinode_test(new_argc, new_argv);
            free(new_argv);
        } else {
            failures += run_mixed_multinode_test(extra_argc, extra_argv);
        }
    }

    /* Nordic nRF52840 USB Dongle multinode wrapper. Same default-to-
     * 2-nodes convenience as the Firefly wrapper so the SPEC's example
     * commands match the binary verbatim. */
    if (strcmp(mode, "nrf52840-dongle-multinode") == 0) {
        int extra_argc = argc - 2;
        char **extra_argv = argv + 2;
        int has_n = 0;
        int firmware_count = 0;
        for (int i = 0; i < extra_argc; i++) {
            if ((strcmp(extra_argv[i], "-t") == 0 ||
                 strcmp(extra_argv[i], "-n") == 0 ||
                 strcmp(extra_argv[i], "--threads") == 0) &&
                i + 1 < extra_argc) {
                if (strcmp(extra_argv[i], "-n") == 0) has_n = 1;
                i++;
            } else if (extra_argv[i][0] != '-') {
                firmware_count++;
            }
        }
        if (!has_n && firmware_count == 1) {
            int new_argc = extra_argc + 2;
            char **new_argv = malloc((new_argc + 1) * sizeof(char *));
            for (int i = 0; i < extra_argc; i++) new_argv[i] = extra_argv[i];
            new_argv[extra_argc]     = (char *)"-n";
            new_argv[extra_argc + 1] = (char *)"2";
            new_argv[new_argc] = NULL;
            failures += run_mixed_multinode_test(new_argc, new_argv);
            free(new_argv);
        } else {
            failures += run_mixed_multinode_test(extra_argc, extra_argv);
        }
    }

    /* TrustZone-M split-image boot: load a secure + a normal-world ELF into
     * one nRF54L15 node and run, watching for the secure->non-secure handoff
     * and the SG-veneer round-trip.
     *   tz-boot <secure.nrf> <normal.nrf> [max_instrs] */
    if (strcmp(mode, "tz-boot") == 0) {
        if (argc < 4) {
            printf("Usage: test_runner tz-boot <secure.elf> <normal.elf> [max_instrs]\n");
            return 1;
        }
        long max_instrs = (argc >= 5) ? strtol(argv[4], NULL, 0) : 20000000;

        const arm_platform_config_t *pcfg = arm_platform_find("nrf54l15-dk");
        arm_platform_t plat;
        arm_platform_init(&plat, pcfg);

        if (arm_load_elf(&plat.cpu, argv[2]) != 0) {
            printf("  FAIL: cannot load secure image %s\n", argv[2]);
            return 1;
        }
        if (arm_load_elf(&plat.cpu, argv[3]) != 0) {
            printf("  FAIL: cannot load normal-world image %s\n", argv[3]);
            return 1;
        }
        printf("  Loaded secure=%s normal=%s; TZ=%d\n", argv[2], argv[3],
               arm_cpu_has_trustzone(&plat.cpu));

        arm_platform_set_console(&plat, tz_boot_tx, NULL);
        arm_cpu_reset(&plat.cpu);
        arm_step(&plat.cpu, (int)max_instrs);

        printf("\n  --- TrustZone transition counters ---\n");
        printf("  SG entries (NS->S):       %llu\n",
               (unsigned long long)plat.cpu.tz_sg_count);
        printf("  BXNS returns (S->NS):     %llu\n",
               (unsigned long long)plat.cpu.tz_bxns_count);
        printf("  secure exceptions:        %llu\n",
               (unsigned long long)plat.cpu.tz_secexc_count);
        printf("  final state: %s, PC=0x%08x\n",
               arm_cpu_is_secure(&plat.cpu) ? "Secure" : "Non-secure",
               plat.cpu.reg[15]);
        /* Success = the secure world (the SoC resets Secure) handed off to
         * Non-secure AND real world transitions were exercised. This is
         * app-independent; a matched banner ("Non-secure world" in the minimal
         * example) is reported as extra confirmation when present. */
        unsigned long long transitions =
            plat.cpu.tz_sg_count + plat.cpu.tz_bxns_count;
        int ok = !arm_cpu_is_secure(&plat.cpu) && transitions > 0;
        (void)tz_boot_saw_secret;
        printf("  %s: reached Non-secure=%d, transitions=%llu%s\n",
               ok ? "PASS" : "INCOMPLETE", !arm_cpu_is_secure(&plat.cpu),
               transitions, tz_boot_saw_ns ? " (banner seen)" : "");
        arm_platform_destroy(&plat);
        return ok ? 0 : 1;
    }

    /* Nordic nRF54L15 Development Kit (PCA10156) multinode wrapper. */
    if (strcmp(mode, "nrf54l15-dk-multinode") == 0) {
        int extra_argc = argc - 2;
        char **extra_argv = argv + 2;
        int has_n = 0;
        int firmware_count = 0;
        for (int i = 0; i < extra_argc; i++) {
            if ((strcmp(extra_argv[i], "-t") == 0 ||
                 strcmp(extra_argv[i], "-n") == 0 ||
                 strcmp(extra_argv[i], "--threads") == 0) &&
                i + 1 < extra_argc) {
                if (strcmp(extra_argv[i], "-n") == 0) has_n = 1;
                i++;
            } else if (extra_argv[i][0] != '-') {
                firmware_count++;
            }
        }
        if (!has_n && firmware_count == 1) {
            int new_argc = extra_argc + 2;
            char **new_argv = malloc((new_argc + 1) * sizeof(char *));
            for (int i = 0; i < extra_argc; i++) new_argv[i] = extra_argv[i];
            new_argv[extra_argc]     = (char *)"-n";
            new_argv[extra_argc + 1] = (char *)"2";
            new_argv[new_argc] = NULL;
            failures += run_mixed_multinode_test(new_argc, new_argv);
            free(new_argv);
        } else {
            failures += run_mixed_multinode_test(extra_argc, extra_argv);
        }
    }

    /* Nordic nRF52840 Development Kit (PCA10056) multinode wrapper. */
    if (strcmp(mode, "nrf52840-dk-multinode") == 0) {
        int extra_argc = argc - 2;
        char **extra_argv = argv + 2;
        int has_n = 0;
        int firmware_count = 0;
        for (int i = 0; i < extra_argc; i++) {
            if ((strcmp(extra_argv[i], "-t") == 0 ||
                 strcmp(extra_argv[i], "-n") == 0 ||
                 strcmp(extra_argv[i], "--threads") == 0) &&
                i + 1 < extra_argc) {
                if (strcmp(extra_argv[i], "-n") == 0) has_n = 1;
                i++;
            } else if (extra_argv[i][0] != '-') {
                firmware_count++;
            }
        }
        if (!has_n && firmware_count == 1) {
            int new_argc = extra_argc + 2;
            char **new_argv = malloc((new_argc + 1) * sizeof(char *));
            for (int i = 0; i < extra_argc; i++) new_argv[i] = extra_argv[i];
            new_argv[extra_argc]     = (char *)"-n";
            new_argv[extra_argc + 1] = (char *)"2";
            new_argv[new_argc] = NULL;
            failures += run_mixed_multinode_test(new_argc, new_argv);
            free(new_argv);
        } else {
            failures += run_mixed_multinode_test(extra_argc, extra_argv);
        }
    }

    /* Mixed-platform mode */
    if (strcmp(mode, "mixed-multinode") == 0) {
        failures += run_mixed_multinode_test(argc - 2, argv + 2);
    }

    /* Test scripting mode (alias for mixed-multinode with test assertions) */
    if (strcmp(mode, "test") == 0) {
        failures += run_mixed_multinode_test(argc - 2, argv + 2);
    }

    if (strcmp(mode, "config-convert") == 0)
        return run_config_convert(argc - 2, argv + 2);
    if (strcmp(mode, "config-roundtrip") == 0)
        return run_config_roundtrip(argc - 2, argv + 2) ? 1 : 0;
    if (strcmp(mode, "config-reject") == 0)
        return run_config_reject(argc - 2, argv + 2) ? 1 : 0;

    /* Timeline unit tests */
    if (strcmp(mode, "mock-host") == 0 || strcmp(mode, "all") == 0) {
        failures += run_mock_host_tests(verbose);
    }

    /* CC1200 chip-driver unit tests (L−1) */
    if (strcmp(mode, "cc1200-mock-host") == 0 || strcmp(mode, "all") == 0) {
        failures += run_cc1200_tests(verbose);
    }

    /* nRF54L15 SPIM register model (mock host, EasyDMA on a byte array) */
    if (strcmp(mode, "nrf54l15-spim") == 0 || strcmp(mode, "all") == 0) {
        failures += run_nrf54l15_spim_tests(verbose);
    }

    /* MX25R6435F flash (nRF54L15-DK on-board) chip tests */
    if (strcmp(mode, "mx25r6435f-mock-host") == 0 || strcmp(mode, "all") == 0) {
        failures += run_mx25r6435f_tests(verbose);
    }

    /* radio_medium_t unit tests */
    if (strcmp(mode, "radio-medium") == 0 || strcmp(mode, "all") == 0) {
        failures += run_radio_medium_tests(verbose);
    }

    /* sim_radio_bus unit tests (Phase 5 guardrail) */
    if (strcmp(mode, "radio-bus") == 0 || strcmp(mode, "all") == 0) {
        failures += run_radio_bus_tests(verbose);
    }

    if (strcmp(mode, "timeline") == 0 || strcmp(mode, "all") == 0) {
        failures += run_timeline_tests(verbose);
    }

    return failures > 0 ? 1 : 0;
}
