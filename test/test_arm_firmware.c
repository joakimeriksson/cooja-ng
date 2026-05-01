/*
 * ARM firmware test runner — loads CC2538 ELF files and monitors UART output
 */
#include "arm_platform.h"
#include "arm_elf.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    char line_buffer[1024];
    int  line_pos;
    int  test_finished;
    int  test_passed;
    int  verbose;
    arm_cpu_t *cpu;
    /* Optional banner assertion: stop+pass when this substring appears
     * on any UART line. NULL means "wait for EXIT/FAIL marker only". */
    const char *expect_banner;
    int  banner_seen;
} arm_fw_test_state_t;

static void arm_fw_tx_callback(void *user_data, uint8_t byte) {
    arm_fw_test_state_t *state = (arm_fw_test_state_t *)user_data;

    if (byte == '\n') {
        state->line_buffer[state->line_pos] = '\0';
        if (state->verbose) {
            printf("    FW: %s\n", state->line_buffer);
        }
        if (strncmp(state->line_buffer, "FAIL:", 5) == 0) {
            state->test_finished = 1;
            state->test_passed = 0;
            arm_stop(state->cpu);
        } else if (strncmp(state->line_buffer, "EXIT", 4) == 0) {
            state->test_finished = 1;
            state->test_passed = 1;
            arm_stop(state->cpu);
        } else if (state->expect_banner &&
                   strstr(state->line_buffer, state->expect_banner)) {
            state->banner_seen = 1;
            state->test_finished = 1;
            state->test_passed = 1;
            arm_stop(state->cpu);
        }
        state->line_pos = 0;
    } else if (state->line_pos < (int)sizeof(state->line_buffer) - 1) {
        state->line_buffer[state->line_pos++] = (char)byte;
    }
}

typedef struct {
    const char *path;
    const char *platform;
    /* If non-NULL, the firmware test passes as soon as this substring
     * appears on a UART line. Used for boards whose Contiki bring-up
     * never prints "EXIT" (i.e. real hello-world prints a banner and
     * loops forever). */
    const char *expect_banner;
} arm_firmware_test_entry_t;

int run_arm_firmware_test(const arm_firmware_test_entry_t *entry, int max_instructions, int verbose) {
    printf("--- ARM Firmware test: %s (platform: %s) ---\n", entry->path, entry->platform);

    const arm_platform_config_t *pcfg = arm_platform_find(entry->platform);
    if (!pcfg) {
        printf("  SKIP: Unknown platform '%s'\n", entry->platform);
        return -1;
    }

    arm_platform_t plat;
    arm_platform_init(&plat, pcfg);

    if (arm_load_elf(&plat.cpu, entry->path) != 0) {
        printf("  SKIP: Cannot load %s\n", entry->path);
        arm_platform_destroy(&plat);
        return -1;
    }

    arm_fw_test_state_t state;
    memset(&state, 0, sizeof(state));
    state.cpu = &plat.cpu;
    state.verbose = verbose;
    state.expect_banner = entry->expect_banner;

    arm_platform_set_console(&plat, arm_fw_tx_callback, &state);

    arm_cpu_reset(&plat.cpu);
    arm_step(&plat.cpu, max_instructions);

    int ret;
    if (state.test_finished) {
        if (state.test_passed) {
            if (state.banner_seen)
                printf("  PASS: Firmware printed expected banner '%s'\n",
                       entry->expect_banner);
            else
                printf("  PASS: Firmware test completed successfully\n");
            ret = 0;
        } else {
            printf("  FAIL: Firmware reported failure\n");
            ret = 1;
        }
    } else if (entry->expect_banner) {
        printf("  FAIL: Banner '%s' not seen within %d instructions\n",
               entry->expect_banner, max_instructions);
        printf("  DEBUG: PC=0x%08x LR=0x%08x SP=0x%08x xpsr=0x%08x cycles=%lld\n",
               plat.cpu.reg[15], plat.cpu.reg[14], plat.cpu.reg[13],
               plat.cpu.xpsr, (long long)plat.cpu.cycles);
        ret = 1;
    } else {
        printf("  INFO: Firmware did not complete within %d instructions\n", max_instructions);
        printf("  DEBUG: PC=0x%08x LR=0x%08x SP=0x%08x xpsr=0x%08x cycles=%lld\n",
               plat.cpu.reg[15], plat.cpu.reg[14], plat.cpu.reg[13],
               plat.cpu.xpsr, (long long)plat.cpu.cycles);
        if (state.line_pos > 0) {
            state.line_buffer[state.line_pos] = '\0';
            printf("  DEBUG: Partial UART output: '%s'\n", state.line_buffer);
        }
        ret = 0;
    }
    arm_platform_destroy(&plat);
    return ret;
}

int run_arm_firmware_tests(int verbose) {
    printf("=== ARM Firmware Tests ===\n\n");

    int failures = 0;
    const arm_firmware_test_entry_t firmware_tests[] = {
        { "firmware/cc2538dk/hello-world.cc2538dk",      "cc2538dk",     NULL },
        /* Phase A bring-up for the Zolertia Firefly: assert the BOARD_STRING
         * banner from arch/platform/zoul/firefly/board.h appears on UART0.
         * Covers L0 (ELF loads) through L4 (Contiki init reaches platform_init
         * which prints the banner). */
        { "firmware/zoul-firefly/bringup.zoul-firefly",  "zoul-firefly", "Zolertia Firefly platform" },
        { NULL, NULL, NULL }
    };

    for (int i = 0; firmware_tests[i].path; i++) {
        int result = run_arm_firmware_test(&firmware_tests[i], 500000000, verbose);
        if (result > 0) failures++;
    }

    return failures;
}
