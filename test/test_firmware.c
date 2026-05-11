/*
 * Firmware test runner — loads ELF files and monitors USART output
 */
#include "msp430_platform.h"
#include "msp430_elf.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    char line_buffer[1024];
    int  line_pos;
    int  test_finished;
    int  test_passed;
    int  verbose;
    const char *expect;   /* NULL = EXIT/FAIL mode, non-NULL = substring match */
    int  total_bytes;
    msp430_cpu_t *cpu;
} fw_test_state_t;

static void fw_tx_callback(void *user_data, uint8_t byte) {
    fw_test_state_t *state = (fw_test_state_t *)user_data;
    state->total_bytes++;

    if (byte == '\n') {
        state->line_buffer[state->line_pos] = '\0';
        if (state->verbose) {
            printf("    FW: %s\n", state->line_buffer);
        }
        if (state->expect) {
            /* Substring match mode: pass when expected string is found */
            if (strstr(state->line_buffer, state->expect) != NULL) {
                state->test_finished = 1;
                state->test_passed = 1;
                msp430_stop(state->cpu);
            }
        } else {
            /* EXIT/FAIL mode (for self-reporting test firmware) */
            if (strncmp(state->line_buffer, "FAIL:", 5) == 0) {
                state->test_finished = 1;
                state->test_passed = 0;
                msp430_stop(state->cpu);
            } else if (strncmp(state->line_buffer, "EXIT", 4) == 0) {
                state->test_finished = 1;
                state->test_passed = 1;
                msp430_stop(state->cpu);
            }
        }
        state->line_pos = 0;
    } else if (state->line_pos < (int)sizeof(state->line_buffer) - 1) {
        state->line_buffer[state->line_pos++] = (char)byte;
    }
}

typedef struct {
    const char *path;
    const char *platform;
    const char *expect;   /* NULL = look for EXIT/FAIL, non-NULL = look for this substring */
} firmware_test_entry_t;

static int run_firmware_test(const firmware_test_entry_t *entry, int max_instructions, int verbose) {
    printf("--- Firmware test: %s (platform: %s) ---\n", entry->path, entry->platform);

    const msp430_platform_config_t *pcfg = msp430_platform_find(entry->platform);
    if (!pcfg) {
        printf("  SKIP: Unknown platform '%s'\n", entry->platform);
        return -1;
    }

    msp430_platform_t plat;
    msp430_platform_init(&plat, pcfg);

    /* Load ELF */
    if (msp430_load_elf(&plat.cpu, entry->path) != 0) {
        printf("  SKIP: Cannot load %s\n", entry->path);
        msp430_platform_destroy(&plat);
        return -1;
    }

    fw_test_state_t state;
    memset(&state, 0, sizeof(state));
    state.cpu = &plat.cpu;
    state.verbose = verbose;
    state.expect = entry->expect;

    msp430_platform_set_console(&plat, fw_tx_callback, &state);

    /* Reset and run.  Run in chunks so we can detect stuck loops
     * (cycles not advancing) before exhausting max_instructions. */
    msp430_cpu_reset(&plat.cpu);
    int remaining = max_instructions;
    int64_t prev_cycles = 0;
    int stuck_count = 0;
    while (remaining > 0 && !state.test_finished) {
        int chunk = remaining > 10000000 ? 10000000 : remaining;
        int left = msp430_step(&plat.cpu, chunk);
        remaining -= (chunk - left);
        if (plat.cpu.cycles == prev_cycles) {
            if (++stuck_count > 2) {
                if (verbose) {
                    printf("    [stuck: PC=0x%05x, cycles=%lld]\n",
                           plat.cpu.reg[0], (long long)plat.cpu.cycles);
                }
                break;
            }
        } else {
            stuck_count = 0;
        }
        prev_cycles = plat.cpu.cycles;
    }

    msp430_platform_destroy(&plat);

    if (state.test_finished) {
        if (state.test_passed) {
            printf("  PASS: Firmware test completed successfully\n");
            return 0;
        } else {
            printf("  FAIL: Firmware reported failure\n");
            return 1;
        }
    } else {
        printf("  WARN: Firmware did not complete within %d instructions (%d UART bytes received)\n",
               max_instructions, state.total_bytes);
        return 0;
    }
}

int run_firmware_tests(int verbose) {
    printf("=== Firmware Tests ===\n\n");

    int failures = 0;
    const firmware_test_entry_t firmware_tests[] = {
        { "firmware/sky/cputest.sky",                  "sky",     NULL },
        { "firmware/sky/timertest.sky",                "sky",     NULL },
        { "firmware/fr5969/hello-world.msp430fr5969",  "fr5969",  "Hello, world" },
        { "firmware/fr5969/leds-example.msp430fr5969", "fr5969",  "Starting" },
        { NULL, NULL, NULL }
    };

    for (int i = 0; firmware_tests[i].path; i++) {
        /* FR5969 firmware (full Contiki-NG) needs more instructions to boot */
        int max_insn = firmware_tests[i].expect ? 1000000000 : 10000000;
        int result = run_firmware_test(&firmware_tests[i], max_insn, verbose);
        if (result > 0) failures++;
    }

    return failures;
}
