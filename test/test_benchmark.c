/*
 * Performance benchmarks for the MSP430 C emulator.
 * Port of PerformanceBenchmark.java
 */
#include "msp430_cpu.h"
#include "msp430_config.h"
#include "msp430_elf.h"
#include "msp430_usart.h"
#include "msp430_timer.h"
#include "msp430_clock.h"
#include "msp430_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FLASH_START   0x4000
#define RAM_START     0x1100
#define WARMUP_ITERS  3
#define MEASURE_ITERS 5
#define INSTRUCTIONS  10000000

#define WW(addr, val) msp430_write_word(&cpu, (addr), (uint16_t)(val))

static double get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

typedef struct {
    const char *name;
    const char *desc;
    long instructions;
    long emulated_cycles;
    double times[MEASURE_ITERS];
    double median_ms;
    double mips;
} bench_result_t;

static bench_result_t results[20];
static int num_results = 0;

static void record_result(const char *name, const char *desc,
                           long instructions, long cycles, double times[]) {
    bench_result_t *r = &results[num_results++];
    r->name = name;
    r->desc = desc;
    r->instructions = instructions;
    r->emulated_cycles = cycles;
    memcpy(r->times, times, sizeof(double) * MEASURE_ITERS);

    double sorted[MEASURE_ITERS];
    memcpy(sorted, times, sizeof(sorted));
    qsort(sorted, MEASURE_ITERS, sizeof(double), cmp_double);
    r->median_ms = sorted[MEASURE_ITERS / 2];
    r->mips = (instructions / 1e6) / (r->median_ms / 1e3);
}

/* ===================================================================
 * Benchmark 1: Register-only ALU loop
 * =================================================================== */
static void bench_register_alu(void) {
    int loop_count = INSTRUCTIONS / 5;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f1611_config);

        int pc = FLASH_START;
        WW(pc, 0x4304); pc += 2;  /* MOV #0, R4 */
        WW(pc, 0x4035); pc += 2;  /* MOV #loopCount, R5 */
        WW(pc, loop_count & 0xffff); pc += 2;

        int loop_start = pc;
        WW(pc, 0x5314); pc += 2;  /* ADD #1, R4 */
        WW(pc, 0x5406); pc += 2;  /* ADD R4, R6 */
        WW(pc, 0x8315); pc += 2;  /* SUB #1, R5 */
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;  /* JNE loop */
        WW(pc, 0x3fff);

        WW(0xfffe, FLASH_START);
        msp430_cpu_reset(&cpu);

        long inst_count = 2 + (long)loop_count * 4 + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 2 + (long)loop_count * 4 + 1;
    record_result("register-alu", "Register-only ADD/SUB/CMP loop", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Benchmark 2: Memory load/store
 * =================================================================== */
static void bench_memory_load_store(void) {
    int loop_count = INSTRUCTIONS / 5;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f1611_config);

        for (int i = 0; i < 256; i++) {
            WW(RAM_START + i * 2, i * 3 + 7);
        }

        int pc = FLASH_START;
        WW(pc, 0x4034); pc += 2;
        WW(pc, RAM_START); pc += 2;
        WW(pc, 0x4036); pc += 2;
        WW(pc, loop_count & 0xffff); pc += 2;

        int loop_start = pc;
        WW(pc, 0x4417); pc += 2;  /* MOV 0(R4), R7 */
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x5417); pc += 2;  /* ADD 2(R4), R7 */
        WW(pc, 0x0002); pc += 2;
        WW(pc, 0x4784); pc += 2;  /* MOV R7, 4(R4) */
        WW(pc, 0x0004); pc += 2;
        WW(pc, 0x8316); pc += 2;  /* SUB #1, R6 */
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x3fff);

        WW(0xfffe, FLASH_START);
        msp430_cpu_reset(&cpu);

        long inst_count = 2 + (long)loop_count * 5 + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 2 + (long)loop_count * 5 + 1;
    record_result("memory-load-store", "Memory load/store with indexed addressing", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Benchmark 3: Branch-heavy workload
 * =================================================================== */
static void bench_branch_heavy(void) {
    int outer_count = INSTRUCTIONS / 29;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f1611_config);

        int pc = FLASH_START;
        WW(pc, 0x4036); pc += 2;
        WW(pc, outer_count & 0xffff); pc += 2;

        int outer_start = pc;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 5); pc += 2;
        WW(pc, 0x4037); pc += 2;
        WW(pc, 0x1234); pc += 2;
        WW(pc, 0x4038); pc += 2;
        WW(pc, 0x5678); pc += 2;

        int inner_start = pc;
        WW(pc, 0x9708); pc += 2;  /* CMP R7, R8 */
        WW(pc, 0x3403); pc += 2;  /* JGE skip */
        WW(pc, 0x4809); pc += 2;
        WW(pc, 0x4708); pc += 2;
        WW(pc, 0x4907); pc += 2;
        /* skip: */
        WW(pc, 0x8314); pc += 2;
        int offset = (inner_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x8316); pc += 2;
        offset = (outer_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x3fff);

        WW(0xfffe, FLASH_START);
        msp430_cpu_reset(&cpu);

        long inst_count = 1 + (long)outer_count * (3 + 5 * 4 + 2) + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 1 + (long)outer_count * (3 + 5 * 4 + 2) + 1;
    record_result("branch-heavy", "Branch-heavy CMP/JGE/JNE pattern", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Benchmark 4: CALL/RET function call overhead
 * =================================================================== */
static void bench_call_return(void) {
    int loop_count = INSTRUCTIONS / 5;
    int stack_top = RAM_START + 0x1000;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f1611_config);

        int pc = FLASH_START;
        WW(pc, 0x4031); pc += 2;
        WW(pc, stack_top); pc += 2;
        WW(pc, 0x4036); pc += 2;
        WW(pc, loop_count & 0xffff); pc += 2;

        int loop_start = pc;
        int func_addr = FLASH_START + 0x100;
        WW(pc, 0x12b0); pc += 2;  /* CALL #func */
        WW(pc, func_addr); pc += 2;
        WW(pc, 0x8316); pc += 2;  /* SUB #1, R6 */
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x3fff);

        pc = func_addr;
        WW(pc, 0x5314); pc += 2;  /* ADD #1, R4 */
        WW(pc, 0x4130);           /* RET */

        WW(0xfffe, FLASH_START);
        msp430_cpu_reset(&cpu);

        long inst_count = 2 + (long)loop_count * 5 + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 2 + (long)loop_count * 5 + 1;
    record_result("call-return", "CALL/RET function call overhead", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Benchmark 5: Byte-mode operations
 * =================================================================== */
static void bench_byte_mode(void) {
    int loop_count = INSTRUCTIONS / 6;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f1611_config);

        int pc = FLASH_START;
        WW(pc, 0x4036); pc += 2;
        WW(pc, loop_count & 0xffff); pc += 2;
        WW(pc, 0x4074); pc += 2;  /* MOV.B #0xAA, R4 */
        WW(pc, 0x00AA); pc += 2;

        int loop_start = pc;
        WW(pc, 0xe074); pc += 2;  /* XOR.B #0xFF, R4 */
        WW(pc, 0x00FF); pc += 2;
        WW(pc, 0x5354); pc += 2;  /* ADD.B #1, R4 (CG) */
        WW(pc, 0xf075); pc += 2;  /* AND.B #0x0F, R5 */
        WW(pc, 0x000F); pc += 2;
        WW(pc, 0xd445); pc += 2;  /* BIS.B R4, R5 */
        WW(pc, 0x8316); pc += 2;  /* SUB #1, R6 */
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x3fff);

        WW(0xfffe, FLASH_START);
        msp430_cpu_reset(&cpu);

        long inst_count = 2 + (long)loop_count * 6 + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 2 + (long)loop_count * 6 + 1;
    record_result("byte-mode", "Byte-mode (.B) ALU operations", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Benchmark 6: Indexed addressing
 * =================================================================== */
static void bench_indexed_addressing(void) {
    int loop_count = INSTRUCTIONS / 5;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f1611_config);

        for (int i = 0; i < 256; i++) {
            WW(RAM_START + i * 2, i);
        }

        int pc = FLASH_START;
        WW(pc, 0x4034); pc += 2;
        WW(pc, RAM_START); pc += 2;
        WW(pc, 0x4036); pc += 2;
        WW(pc, loop_count & 0xffff); pc += 2;

        int loop_start = pc;
        WW(pc, 0x4417); pc += 2;
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x5417); pc += 2;
        WW(pc, 0x0002); pc += 2;
        WW(pc, 0x4784); pc += 2;
        WW(pc, 0x0004); pc += 2;
        WW(pc, 0x8316); pc += 2;
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x3fff);

        WW(0xfffe, FLASH_START);
        msp430_cpu_reset(&cpu);

        long inst_count = 2 + (long)loop_count * 5 + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 2 + (long)loop_count * 5 + 1;
    record_result("indexed-addressing", "Indexed X(Rn) addressing mode", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Benchmark 7: MSP430X extended instructions
 * =================================================================== */
static void bench_msp430x(void) {
    int loop_count = INSTRUCTIONS / 4;
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, &msp430f5437_config);

        int entry_point = 0x5c00;
        int pc = entry_point;

        WW(pc, 0x0086); pc += 2;  /* MOVA #loopCount, R6 */
        WW(pc, loop_count & 0xffff); pc += 2;

        int loop_start = pc;
        WW(pc, 0x00a4); pc += 2;  /* ADDA #1, R4 */
        WW(pc, 0x0001); pc += 2;
        WW(pc, 0x00b6); pc += 2;  /* SUBA #1, R6 */
        WW(pc, 0x0001); pc += 2;
        WW(pc, 0x0096); pc += 2;  /* CMPA #0, R6 */
        WW(pc, 0x0000); pc += 2;
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;
        WW(pc, 0x3fff);

        WW(0xfffe, entry_point);
        msp430_cpu_reset(&cpu);

        long inst_count = 1 + (long)loop_count * 4 + 1;
        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    long inst_count = 1 + (long)loop_count * 4 + 1;
    record_result("msp430x-extended", "MSP430X 20-bit MOVA/ADDA/SUBA", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Firmware benchmark helper — sets up peripherals and runs firmware
 * =================================================================== */

/* SFR state for firmware benchmarks */
static struct { uint8_t ie1, ie2, ifg1, ifg2; } bench_sfr;

static int bench_sfr_read(void *d, uint32_t a, bool w, int64_t c) {
    (void)d;(void)w;(void)c;
    switch(a) { case 0: return bench_sfr.ie1; case 1: return bench_sfr.ie2;
                case 2: return bench_sfr.ifg1; case 3: return bench_sfr.ifg2; }
    return 0;
}
static void bench_sfr_write(void *d, uint32_t a, int v, bool w, int64_t c) {
    (void)d;(void)w;(void)c;
    switch(a) { case 0: bench_sfr.ie1=v; break; case 1: bench_sfr.ie2=v; break;
                case 2: bench_sfr.ifg1=v; break; case 3: bench_sfr.ifg2=v; break; }
}
static void bench_tx_noop(void *d, uint8_t b) { (void)d; (void)b; }

static void bench_firmware(const char *name, const char *path) {
    double times[MEASURE_ITERS];
    long emulated_cycles = 0;
    long inst_count = INSTRUCTIONS;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        const msp430_config_t *cfg = &msp430f1611_config;
        msp430_cpu_t cpu;
        msp430_cpu_init(&cpu, cfg);

        if (msp430_load_elf(&cpu, path) != 0) {
            printf("SKIP (cannot load %s)\n", path);
            return;
        }

        memset(&bench_sfr, 0, sizeof(bench_sfr));
        msp430_register_io(&cpu, 0, 4, bench_sfr_read, bench_sfr_write, NULL);

        msp430_clock_t clock;
        msp430_clock_init(&clock, &cpu, cfg->bcs_base, cfg->max_dco_freq);

        msp430_timer_t timer_a, timer_b;
        msp430_timer_init(&timer_a, &cpu, &clock, "Timer_A3",
                           cfg->timer_a_base, cfg->timer_a_iv,
                           cfg->timer_a_num_ccr, cfg->timer_a_ccr0_vec,
                           cfg->timer_a_ccr1_vec);
        msp430_clock_add_timer(&clock, &timer_a);

        msp430_timer_init(&timer_b, &cpu, &clock, "Timer_B7",
                           cfg->timer_b_base, cfg->timer_b_iv,
                           cfg->timer_b_num_ccr, cfg->timer_b_ccr0_vec,
                           cfg->timer_b_ccr1_vec);
        msp430_clock_add_timer(&clock, &timer_b);

        msp430_gpio_t gpio;
        msp430_gpio_init(&gpio, &cpu);
        msp430_gpio_add_port(&gpio, 4, 0x1C, -1);

        msp430_usart_t usart0, usart1;
        msp430_usart_init(&usart0, &cpu, cfg->usart0_base, cfg->usart_tx_offset);
        msp430_usart_init(&usart1, &cpu, cfg->usart1_base, cfg->usart_tx_offset);
        msp430_usart_set_callback(&usart0, bench_tx_noop, NULL);
        msp430_usart_set_callback(&usart1, bench_tx_noop, NULL);
        msp430_usart_set_ifg(&usart0, &bench_sfr.ifg1, 0x80);
        msp430_usart_set_ifg(&usart1, &bench_sfr.ifg2, 0x20);

        msp430_cpu_reset(&cpu);

        int64_t start_cycles = cpu.cycles;
        double start = get_time_ms();
        msp430_step(&cpu, (int)inst_count);
        double elapsed = get_time_ms() - start;
        emulated_cycles = cpu.cycles - start_cycles;

        if (iter >= 0) times[iter] = elapsed;
        msp430_cpu_destroy(&cpu);
    }

    record_result(name, "Firmware benchmark", inst_count, emulated_cycles, times);
}

/* ===================================================================
 * Print results
 * =================================================================== */
static void print_results(void) {
    printf("\n=== Results ===\n\n");
    printf("%-25s %10s %10s %10s %12s\n",
           "Benchmark", "Median ms", "Min ms", "Max ms", "MIPS");
    printf("%-25s %10s %10s %10s %12s\n",
           "-------------------------", "----------", "----------", "----------", "------------");

    for (int i = 0; i < num_results; i++) {
        bench_result_t *r = &results[i];
        double sorted[MEASURE_ITERS];
        memcpy(sorted, r->times, sizeof(sorted));
        qsort(sorted, MEASURE_ITERS, sizeof(double), cmp_double);

        printf("%-25s %10.1f %10.1f %10.1f %12.1f\n",
               r->name, sorted[MEASURE_ITERS / 2], sorted[0],
               sorted[MEASURE_ITERS - 1], r->mips);
    }

    printf("\n=== Machine-Readable Summary (CSV) ===\n");
    printf("benchmark,median_ms,mips,instructions,emulated_cycles\n");
    for (int i = 0; i < num_results; i++) {
        bench_result_t *r = &results[i];
        printf("%s,%.1f,%.1f,%ld,%ld\n",
               r->name, r->median_ms, r->mips, r->instructions, r->emulated_cycles);
    }
}

/* ===================================================================
 * Run all benchmarks
 * =================================================================== */
int run_benchmarks(void) {
    num_results = 0;

    printf("=== MSP430 C Emulator Performance Benchmarks ===\n");
    printf("Warmup: %d, Measured: %d, Instructions/iter: %d\n\n", WARMUP_ITERS, MEASURE_ITERS, INSTRUCTIONS);

    printf("Running register-alu... "); fflush(stdout);
    bench_register_alu(); printf("done.\n");

    printf("Running memory-load-store... "); fflush(stdout);
    bench_memory_load_store(); printf("done.\n");

    printf("Running branch-heavy... "); fflush(stdout);
    bench_branch_heavy(); printf("done.\n");

    printf("Running call-return... "); fflush(stdout);
    bench_call_return(); printf("done.\n");

    printf("Running byte-mode... "); fflush(stdout);
    bench_byte_mode(); printf("done.\n");

    printf("Running indexed-addressing... "); fflush(stdout);
    bench_indexed_addressing(); printf("done.\n");

    printf("Running msp430x-extended... "); fflush(stdout);
    bench_msp430x(); printf("done.\n");

    printf("Running firmware blink.sky... "); fflush(stdout);
    bench_firmware("fw-blink", "firmware/sky/blink.sky"); printf("done.\n");

    printf("Running firmware energest-demo.sky... "); fflush(stdout);
    bench_firmware("fw-energest-demo", "firmware/sky/energest-demo.sky"); printf("done.\n");

    print_results();
    return 0;
}
