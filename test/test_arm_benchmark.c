/*
 * Performance benchmarks for the ARM Cortex-M interpreter.
 *
 * Companion to test_benchmark.c (MSP430).  Exists because there was no ARM
 * benchmark at all: `bench` is MSP430-only, so every ARM performance figure
 * had to be hand-timed and nothing could gate a regression.  See
 * docs/design/arm-performance-plan.md §1.
 *
 * Primary metric is MIPS (engine throughput).  x-real-time is reported as a
 * secondary number and must be read with care: it is sensitive to idle-skip
 * policy, not just interpreter speed, so it is only meaningful for the
 * firmware cases where the workload is fixed.
 *
 * The synthetic cases each isolate one hot path:
 *   alu-reg      Thumb-16 register ALU  -> computed-goto dispatch
 *   thumb2-dp    Thumb-2 32-bit dataproc -> the t32_decode switch nest
 *   it-block     IT blocks               -> condition_passed
 *   branch       conditional branches    -> condition_passed + PC writes
 *   mem-ldr-str  load/store              -> arm_read32/arm_write32
 *
 * Every synthetic case is *guarded* by an iteration-count invariant: r7 is
 * incremented exactly once per loop pass and touched by nothing else, so after
 * a run of N instructions r7 must equal N / instructions-per-iteration.  A
 * corrupted or unimplemented encoding changes the loop length or diverts
 * control flow, and the count no longer matches -> reported as BROKEN rather
 * than silently timed.  A benchmark that measures garbage is worse than no
 * benchmark.
 *
 * Both guards were validated by injection rather than assumed:
 *   - Deleting one instruction from the alu-reg loop (declared 10/iter, actual
 *     9) trips it: iters=2222222 vs expected=2000000.
 *   - It also caught a real mistake while this file was being written — the
 *     mem-ldr-str loop was declared at 6 instructions/iteration when it is 7.
 *
 * Known limit, stated so the guard is not over-trusted: it detects changes to
 * loop *length* or *control flow*, not a wrong instruction that happens to be
 * flow-neutral.  Injecting 0xDEAD, for instance, is not caught — this emulator
 * executes it as a silent no-op rather than faulting, so the loop keeps its
 * shape.  Semantic correctness of the encodings is the job of
 * arm-correctness, not of this benchmark.
 */
#include "arm_cpu.h"
#include "arm_config.h"
#include "arm_elf.h"
#include "arm_platform.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define WARMUP_ITERS   2
#define MEASURE_ITERS  5
#define INSTRUCTIONS   20000000     /* per measured iteration */
#define FW_INSTRUCTIONS 20000000

#define CODE_BASE (ARM_FLASH_BASE + 0x100)

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
    long   instructions;
    long   emulated_cycles;
    double times[MEASURE_ITERS];
    double median_ms;
    double mips;
    double x_realtime;          /* emulated seconds / wall seconds */
    int    broken;              /* guard tripped -> do not trust the number */
} arm_bench_result_t;

static arm_bench_result_t results[20];
static int num_results = 0;

static void record_result(const char *name, const char *desc,
                          long instructions, long cycles, uint32_t freq_hz,
                          double times[], int broken) {
    arm_bench_result_t *r = &results[num_results++];
    r->name = name;
    r->desc = desc;
    r->instructions = instructions;
    r->emulated_cycles = cycles;
    r->broken = broken;
    memcpy(r->times, times, sizeof(double) * MEASURE_ITERS);

    double sorted[MEASURE_ITERS];
    memcpy(sorted, times, sizeof(sorted));
    qsort(sorted, MEASURE_ITERS, sizeof(double), cmp_double);
    r->median_ms = sorted[MEASURE_ITERS / 2];
    r->mips = (instructions / 1e6) / (r->median_ms / 1e3);
    double emu_seconds = freq_hz ? (double)cycles / (double)freq_hz : 0.0;
    r->x_realtime = emu_seconds / (r->median_ms / 1e3);
}

/* --- Thumb code emitter ------------------------------------------------ */

static void write_flash32(arm_cpu_t *cpu, uint32_t addr, uint32_t val) {
    uint32_t off = addr - ARM_FLASH_BASE;
    cpu->flash[off]     = val & 0xFF;
    cpu->flash[off + 1] = (val >> 8) & 0xFF;
    cpu->flash[off + 2] = (val >> 16) & 0xFF;
    cpu->flash[off + 3] = (val >> 24) & 0xFF;
}

static void emit16(arm_cpu_t *cpu, uint32_t *pc, uint16_t insn) {
    uint32_t off = *pc - ARM_FLASH_BASE;
    cpu->flash[off]     = insn & 0xFF;
    cpu->flash[off + 1] = (insn >> 8) & 0xFF;
    *pc += 2;
}

static void emit32(arm_cpu_t *cpu, uint32_t *pc, uint16_t hw1, uint16_t hw2) {
    emit16(cpu, pc, hw1);
    emit16(cpu, pc, hw2);
}

/* Unconditional backward branch (T2): target = PC_of_B + 4 + offset. */
static void emit_b_back(arm_cpu_t *cpu, uint32_t *pc, uint32_t target) {
    int32_t off = (int32_t)target - (int32_t)(*pc + 4);
    uint16_t imm11 = (uint16_t)((off >> 1) & 0x7FF);
    emit16(cpu, pc, (uint16_t)(0xE000 | imm11));
}

/* Conditional forward branch (T1): target = PC_of_B + 4 + offset. */
static void emit_bcond_fwd(arm_cpu_t *cpu, uint32_t *pc, int cond,
                           uint32_t target) {
    int32_t off = (int32_t)target - (int32_t)(*pc + 4);
    uint8_t imm8 = (uint8_t)((off >> 1) & 0xFF);
    emit16(cpu, pc, (uint16_t)(0xD000 | ((cond & 0xF) << 8) | imm8));
}

static void setup_cpu(arm_cpu_t *cpu) {
    arm_cpu_init(cpu, &cc2538_config);
    write_flash32(cpu, ARM_FLASH_BASE, ARM_SRAM_BASE + ARM_SRAM_SIZE);
    write_flash32(cpu, ARM_FLASH_BASE + 4, CODE_BASE + 1);  /* thumb bit */
    arm_cpu_reset(cpu);
}

/* Emitter callback: writes the loop, returns the address of the loop top and
 * (via loop_end) the first address past the loop body. */
typedef uint32_t (*emit_fn)(arm_cpu_t *cpu, uint32_t *pc, uint32_t *loop_end,
                            int *insns_per_iter);

/* Iteration counter shared by every loop: ADDS r7, #1.  r7 is written by
 * nothing else, so it is an exact pass counter. */
#define EMIT_ITER_COUNTER(cpu, pc) emit16((cpu), (pc), 0x3701)

/* --- The synthetic loops ---------------------------------------------- */

/* Thumb-16 register-only ALU: 8 ALU ops + branch per iteration. */
static uint32_t emit_alu_reg(arm_cpu_t *cpu, uint32_t *pc, uint32_t *loop_end,
                             int *ipi) {
    emit16(cpu, pc, 0x2000);            /* MOVS r0, #0  */
    emit16(cpu, pc, 0x2101);            /* MOVS r1, #1  */
    emit16(cpu, pc, 0x2202);            /* MOVS r2, #2  */
    emit16(cpu, pc, 0x2303);            /* MOVS r3, #3  */
    emit16(cpu, pc, 0x2700);            /* MOVS r7, #0  */
    uint32_t loop = *pc;
    EMIT_ITER_COUNTER(cpu, pc);
    emit16(cpu, pc, 0x1840);            /* ADDS r0, r0, r1 */
    emit16(cpu, pc, 0x1880);            /* ADDS r0, r0, r2 */
    emit16(cpu, pc, 0x4058);            /* EORS r0, r3     */
    emit16(cpu, pc, 0x4008);            /* ANDS r0, r1     */
    emit16(cpu, pc, 0x4310);            /* ORRS r0, r2     */
    emit16(cpu, pc, 0x1A40);            /* SUBS r0, r0, r1 */
    emit16(cpu, pc, 0x0040);            /* LSLS r0, r0, #1 */
    emit16(cpu, pc, 0x0840);            /* LSRS r0, r0, #1 */
    emit_b_back(cpu, pc, loop);
    *loop_end = *pc;
    *ipi = 10;   /* counter + 8 ALU + branch */
    return loop;
}

/* Thumb-2 32-bit data processing: every instruction takes t32_decode. */
static uint32_t emit_thumb2_dp(arm_cpu_t *cpu, uint32_t *pc, uint32_t *loop_end,
                               int *ipi) {
    emit16(cpu, pc, 0x2000);            /* MOVS r0, #0 */
    emit16(cpu, pc, 0x2101);            /* MOVS r1, #1 */
    emit16(cpu, pc, 0x2700);            /* MOVS r7, #0 */
    uint32_t loop = *pc;
    EMIT_ITER_COUNTER(cpu, pc);
    emit32(cpu, pc, 0xF100, 0x0001);    /* ADD.W  r0, r0, #1  */
    emit32(cpu, pc, 0xEA40, 0x0001);    /* ORR.W  r0, r0, r1  */
    emit32(cpu, pc, 0xEA00, 0x0001);    /* AND.W  r0, r0, r1  */
    emit32(cpu, pc, 0xEA80, 0x0001);    /* EOR.W  r0, r0, r1  */
    emit32(cpu, pc, 0xF1A0, 0x0001);    /* SUB.W  r0, r0, #1  */
    emit32(cpu, pc, 0xEB00, 0x0001);    /* ADD.W  r0, r0, r1  */
    emit_b_back(cpu, pc, loop);
    *loop_end = *pc;
    *ipi = 8;    /* counter + 6 dataproc + branch */
    return loop;
}

/* IT blocks: exercises condition_passed on every guarded instruction. */
static uint32_t emit_it_block(arm_cpu_t *cpu, uint32_t *pc, uint32_t *loop_end,
                              int *ipi) {
    emit16(cpu, pc, 0x2000);            /* MOVS r0, #0 */
    emit16(cpu, pc, 0x2700);            /* MOVS r7, #0 */
    uint32_t loop = *pc;
    EMIT_ITER_COUNTER(cpu, pc);
    emit16(cpu, pc, 0x2800);            /* CMP  r0, #0 */
    emit16(cpu, pc, 0xBF08);            /* IT   EQ     */
    emit16(cpu, pc, 0x3001);            /* ADDEQ r0, #1 */
    emit16(cpu, pc, 0x2800);            /* CMP  r0, #0 */
    emit16(cpu, pc, 0xBF18);            /* IT   NE     */
    emit16(cpu, pc, 0x3801);            /* SUBNE r0, #1 */
    emit16(cpu, pc, 0x2801);            /* CMP  r0, #1 */
    emit16(cpu, pc, 0xBF08);            /* IT   EQ     */
    emit16(cpu, pc, 0x3002);            /* ADDEQ r0, #2 */
    emit_b_back(cpu, pc, loop);
    *loop_end = *pc;
    *ipi = 11;   /* counter + 3x(CMP+IT+op) + branch */
    return loop;
}

/* Conditional branches: one taken, one not-taken per iteration. */
static uint32_t emit_branch(arm_cpu_t *cpu, uint32_t *pc, uint32_t *loop_end,
                            int *ipi) {
    emit16(cpu, pc, 0x2000);            /* MOVS r0, #0 */
    emit16(cpu, pc, 0x2100);            /* MOVS r1, #0 */
    emit16(cpu, pc, 0x2200);            /* MOVS r2, #0 */
    emit16(cpu, pc, 0x2700);            /* MOVS r7, #0 */
    uint32_t loop = *pc;
    EMIT_ITER_COUNTER(cpu, pc);
    emit16(cpu, pc, 0x3001);            /* ADDS r0, #1 */
    emit16(cpu, pc, 0x2800);            /* CMP  r0, #0 */
    /* BEQ over one instruction (not taken in the common case) */
    uint32_t bp1 = *pc; *pc += 2;       /* placeholder */
    emit16(cpu, pc, 0x3101);            /* ADDS r1, #1 */
    uint32_t skip1 = *pc;
    { uint32_t t = bp1; emit_bcond_fwd(cpu, &t, 0 /*EQ*/, skip1); }
    emit16(cpu, pc, 0x2800);            /* CMP  r0, #0 */
    uint32_t bp2 = *pc; *pc += 2;
    emit16(cpu, pc, 0x3201);            /* ADDS r2, #1 */
    uint32_t skip2 = *pc;
    { uint32_t t = bp2; emit_bcond_fwd(cpu, &t, 1 /*NE*/, skip2); }
    emit_b_back(cpu, pc, loop);
    *loop_end = *pc;
    /* Steady state (r0 != 0): BEQ not taken so ADDS r1 runs; BNE taken so
     * ADDS r2 is skipped.  counter + ADDS r0 + CMP + BEQ + ADDS r1 + CMP
     * + BNE + B = 8.  Only the first pass differs (r0 == 0). */
    *ipi = 8;
    return loop;
}

/* Load/store against SRAM: exercises the memory path. */
static uint32_t emit_mem(arm_cpu_t *cpu, uint32_t *pc, uint32_t *loop_end,
                         int *ipi) {
    emit16(cpu, pc, 0x2000);            /* MOVS r0, #0    */
    emit16(cpu, pc, 0x2120);            /* MOVS r1, #0x20 */
    emit16(cpu, pc, 0x0609);            /* LSLS r1, r1, #24 -> 0x20000000 */
    emit16(cpu, pc, 0x2700);            /* MOVS r7, #0    */
    uint32_t loop = *pc;
    EMIT_ITER_COUNTER(cpu, pc);
    emit16(cpu, pc, 0x6008);            /* STR  r0, [r1, #0] */
    emit16(cpu, pc, 0x680A);            /* LDR  r2, [r1, #0] */
    emit16(cpu, pc, 0x604A);            /* STR  r2, [r1, #4] */
    emit16(cpu, pc, 0x684B);            /* LDR  r3, [r1, #4] */
    emit16(cpu, pc, 0x3001);            /* ADDS r0, #1       */
    emit_b_back(cpu, pc, loop);
    *loop_end = *pc;
    *ipi = 7;    /* counter + 2 STR + 2 LDR + ADDS + branch */
    return loop;
}

/* --- Runner ------------------------------------------------------------ */

static void bench_synth(const char *name, const char *desc, emit_fn emit) {
    double times[MEASURE_ITERS];
    long cycles = 0;
    uint32_t freq = 0;
    int broken = 0;

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        arm_cpu_t cpu;
        setup_cpu(&cpu);

        uint32_t pc = CODE_BASE, loop_end = 0;
        int ipi = 0;
        uint32_t loop_top = emit(&cpu, &pc, &loop_end, &ipi);

        int64_t c0 = cpu.cycles, i0 = cpu.instructions;
        double start = get_time_ms();
        arm_step(&cpu, INSTRUCTIONS);
        double elapsed = get_time_ms() - start;

        /* Guards, strongest first.
         * (a) Iteration invariant: r7 counts loop passes and nothing else
         *     writes it, so it must equal INSTRUCTIONS/ipi.  This is what
         *     actually detects a wrong or unimplemented encoding — the loop
         *     length or the control flow changes and the count drifts.
         * (b) PC still inside the body, and (c) the full budget retired:
         *     cheap corroboration, insufficient on their own. */
        uint32_t iters = cpu.reg[7];
        long expect_iters = (long)INSTRUCTIONS / (ipi ? ipi : 1);
        long delta = (long)iters - expect_iters;
        if (delta < -4 || delta > 4)                    broken |= 1;
        uint32_t final_pc = cpu.reg[ARM_PC];
        if (final_pc < loop_top || final_pc > loop_end) broken |= 2;
        if (cpu.instructions - i0 != INSTRUCTIONS)      broken |= 4;
        if (broken && iter == MEASURE_ITERS - 1)
            printf("\n    [guard] %s: iters=%u expected=%ld pc=0x%x "
                   "body=[0x%x,0x%x] flags=0x%x\n",
                   name, iters, expect_iters, final_pc, loop_top, loop_end,
                   broken);

        cycles = (long)(cpu.cycles - c0);
        freq = cpu.cpu_freq_hz;
        if (iter >= 0) times[iter] = elapsed;
        arm_cpu_destroy(&cpu);
    }

    record_result(name, desc, INSTRUCTIONS, cycles, freq, times, broken);
}

static void fw_tx_noop(void *d, uint8_t b) { (void)d; (void)b; }

static int bench_firmware(const char *name, const char *platform,
                          const char *path) {
    double times[MEASURE_ITERS];
    long cycles = 0;
    uint32_t freq = 0;

    const arm_platform_config_t *pcfg = arm_platform_find(platform);
    if (!pcfg) {
        printf("SKIP (no platform %s)\n", platform);
        return 1;
    }

    for (int iter = -WARMUP_ITERS; iter < MEASURE_ITERS; iter++) {
        arm_platform_t plat;
        arm_platform_init(&plat, pcfg);
        if (arm_load_elf(&plat.cpu, path) != 0) {
            printf("SKIP (cannot load %s)\n", path);
            arm_platform_destroy(&plat);
            return 1;
        }
        arm_platform_set_console(&plat, fw_tx_noop, NULL);
        arm_cpu_reset(&plat.cpu);

        int64_t c0 = plat.cpu.cycles;
        double start = get_time_ms();
        arm_step(&plat.cpu, FW_INSTRUCTIONS);
        double elapsed = get_time_ms() - start;

        cycles = (long)(plat.cpu.cycles - c0);
        freq = plat.cpu.cpu_freq_hz;
        if (iter >= 0) times[iter] = elapsed;
        arm_platform_destroy(&plat);
    }

    record_result(name, "Firmware benchmark", FW_INSTRUCTIONS, cycles, freq,
                  times, 0);
    return 0;
}

/* --- Reporting --------------------------------------------------------- */

static void print_results(void) {
    printf("\n=== Results ===\n\n");
    printf("%-20s %10s %10s %10s %12s %10s\n",
           "Benchmark", "Median ms", "Min ms", "Max ms", "MIPS", "xRealtime");
    printf("%-20s %10s %10s %10s %12s %10s\n",
           "--------------------", "----------", "----------", "----------",
           "------------", "----------");

    for (int i = 0; i < num_results; i++) {
        arm_bench_result_t *r = &results[i];
        double sorted[MEASURE_ITERS];
        memcpy(sorted, r->times, sizeof(sorted));
        qsort(sorted, MEASURE_ITERS, sizeof(double), cmp_double);

        printf("%-20s %10.1f %10.1f %10.1f %12.1f %10.1f%s\n",
               r->name, sorted[MEASURE_ITERS / 2], sorted[0],
               sorted[MEASURE_ITERS - 1], r->mips, r->x_realtime,
               r->broken ? "   <-- BROKEN" : "");
    }

    printf("\n=== Machine-Readable Summary (CSV) ===\n");
    printf("benchmark,median_ms,mips,x_realtime,instructions,emulated_cycles,broken\n");
    for (int i = 0; i < num_results; i++) {
        arm_bench_result_t *r = &results[i];
        printf("%s,%.1f,%.1f,%.1f,%ld,%ld,%d\n",
               r->name, r->median_ms, r->mips, r->x_realtime,
               r->instructions, r->emulated_cycles, r->broken);
    }
}

int run_arm_benchmarks(void);

int run_arm_benchmarks(void) {
    num_results = 0;

    printf("=== ARM Cortex-M Interpreter Performance Benchmarks ===\n");
    printf("Warmup: %d, Measured: %d, Instructions/iter: %d\n\n",
           WARMUP_ITERS, MEASURE_ITERS, INSTRUCTIONS);

    printf("Running alu-reg... ");     fflush(stdout);
    bench_synth("alu-reg", "Thumb-16 register ALU", emit_alu_reg);
    printf("done.\n");

    printf("Running thumb2-dp... ");   fflush(stdout);
    bench_synth("thumb2-dp", "Thumb-2 32-bit dataproc", emit_thumb2_dp);
    printf("done.\n");

    printf("Running it-block... ");    fflush(stdout);
    bench_synth("it-block", "IT-block conditionals", emit_it_block);
    printf("done.\n");

    printf("Running branch... ");      fflush(stdout);
    bench_synth("branch", "Conditional branches", emit_branch);
    printf("done.\n");

    printf("Running mem-ldr-str... "); fflush(stdout);
    bench_synth("mem-ldr-str", "Load/store", emit_mem);
    printf("done.\n");

    printf("Running fw-zephyr-sync... "); fflush(stdout);
    if (bench_firmware("fw-zephyr-sync", "nrf52840-dk",
                       "firmware/nrf52840-dk/zephyr-synchronization.nrf52840-dk") == 0)
        printf("done.\n");

    printf("Running fw-cc2538-udp... "); fflush(stdout);
    if (bench_firmware("fw-cc2538-udp", "cc2538dk",
                       "firmware/cc2538dk/udp-server.cc2538dk") == 0)
        printf("done.\n");

    print_results();

    int broken = 0;
    for (int i = 0; i < num_results; i++) broken += results[i].broken ? 1 : 0;
    if (broken) {
        printf("\n%d benchmark(s) BROKEN: the loop faulted or escaped; "
               "their timings are meaningless.\n", broken);
        return broken;
    }
    return 0;
}
