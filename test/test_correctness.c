/*
 * Correctness tests for the MSP430 CPU emulator.
 * Direct port of CorrectnessTests.java — all 70 tests.
 */
#include "msp430_cpu.h"
#include "msp430_config.h"
#include <stdio.h>
#include <string.h>

/* Test framework */
static int passed = 0;
static int failed = 0;
static int verbose = 0;

void test_correctness_set_verbose(int v) { verbose = v; }

static void assert_eq(const char *name, long expected, long actual) {
    if (expected == actual) {
        passed++;
        if (verbose) printf("  PASS: %s\n", name);
    } else {
        failed++;
        printf("  FAIL: %s (expected 0x%lx (%ld), got 0x%lx (%ld))\n",
               name, expected, expected, actual, actual);
    }
}

static void assert_true(const char *name, int condition) {
    if (condition) {
        passed++;
        if (verbose) printf("  PASS: %s\n", name);
    } else {
        failed++;
        printf("  FAIL: %s\n", name);
    }
}

/* Helper: create and reset CPU */
static void setup_cpu(msp430_cpu_t *cpu, const msp430_config_t *config, int entry_point) {
    msp430_cpu_init(cpu, config);
    msp430_write_word(cpu, 0xfffe, (uint16_t)entry_point);
    msp430_cpu_reset(cpu);
}

static void cleanup_cpu(msp430_cpu_t *cpu) {
    msp430_cpu_destroy(cpu);
}

#define WW(addr, val) msp430_write_word(&cpu, (addr), (uint16_t)(val))
#define RW(addr) msp430_read_word(&cpu, (addr))

/* ===================================================================
 * MOV instruction tests
 * =================================================================== */
static void test_mov(void) {
    printf("--- MOV instruction tests ---\n");

    /* MOV #imm, Rd */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;  /* MOV #0x1234, R4 */
        WW(pc, 0x1234); pc += 2;
        WW(pc, 0x4035); pc += 2;  /* MOV #0xABCD, R5 */
        WW(pc, 0xABCD); pc += 2;
        WW(pc, 0x3fff);           /* JMP $ */
        msp430_step(&cpu, 2);
        assert_eq("MOV #0x1234,R4", 0x1234, cpu.reg[4]);
        assert_eq("MOV #0xABCD,R5", 0xABCD, cpu.reg[5]);
        cleanup_cpu(&cpu);
    }

    /* MOV Rn, Rm */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x5555); pc += 2;
        WW(pc, 0x4405); pc += 2;  /* MOV R4, R5 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("MOV R4,R5", 0x5555, cpu.reg[5]);
        cleanup_cpu(&cpu);
    }

    /* MOV Rn, X(Rm) — store to memory */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;  /* MOV #RAM_START, R4 */
        WW(pc, 0x1100); pc += 2;
        WW(pc, 0x4035); pc += 2;  /* MOV #0xBEEF, R5 */
        WW(pc, 0xBEEF); pc += 2;
        WW(pc, 0x4584); pc += 2;  /* MOV R5, 0(R4) */
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("MOV R5,0(R4) -> RAM", 0xBEEF, RW(0x1100));
        cleanup_cpu(&cpu);
    }

    /* MOV using CG constants */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;  /* MOV #0, R4 (CG2 As=00) */
        WW(pc, 0x4315); pc += 2;  /* MOV #1, R5 (CG2 As=01) */
        WW(pc, 0x4326); pc += 2;  /* MOV #2, R6 (CG2 As=10) */
        WW(pc, 0x4337); pc += 2;  /* MOV #-1, R7 (CG2 As=11) */
        WW(pc, 0x4228); pc += 2;  /* MOV #4, R8 (CG1 As=10) */
        WW(pc, 0x4239); pc += 2;  /* MOV #8, R9 (CG1 As=11) */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 6);
        assert_eq("MOV #0 (CG),R4", 0, cpu.reg[4]);
        assert_eq("MOV #1 (CG),R5", 1, cpu.reg[5]);
        assert_eq("MOV #2 (CG),R6", 2, cpu.reg[6]);
        assert_eq("MOV #-1 (CG),R7", 0xFFFF, cpu.reg[7] & 0xFFFF);
        assert_eq("MOV #4 (CG),R8", 4, cpu.reg[8]);
        assert_eq("MOV #8 (CG),R9", 8, cpu.reg[9]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Arithmetic tests
 * =================================================================== */
static void test_arithmetic(void) {
    printf("--- Arithmetic instruction tests ---\n");

    /* ADD #imm, Rd */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;  /* MOV #100, R4 */
        WW(pc, 100); pc += 2;
        WW(pc, 0x5034); pc += 2;  /* ADD #200, R4 */
        WW(pc, 200); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("ADD #200 + 100 = 300", 300, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* ADD overflow sets carry */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0xFFFF); pc += 2;
        WW(pc, 0x5314); pc += 2;  /* ADD #1, R4 (CG) */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("ADD 0xFFFF+1 => 0", 0, cpu.reg[4] & 0xFFFF);
        assert_true("ADD overflow sets CARRY", (cpu.reg[MSP430_SR] & SR_C) != 0);
        assert_true("ADD overflow sets ZERO", (cpu.reg[MSP430_SR] & SR_Z) != 0);
        cleanup_cpu(&cpu);
    }

    /* SUB */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 500); pc += 2;
        WW(pc, 0x8034); pc += 2;  /* SUB #200, R4 */
        WW(pc, 200); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("SUB 500-200 = 300", 300, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* SUB underflow */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 5); pc += 2;
        WW(pc, 0x8034); pc += 2;  /* SUB #10, R4 */
        WW(pc, 10); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("SUB 5-10 = 0xFFFB (-5)", 0xFFFB, cpu.reg[4] & 0xFFFF);
        assert_true("SUB underflow sets NEGATIVE", (cpu.reg[MSP430_SR] & SR_N) != 0);
        cleanup_cpu(&cpu);
    }

    /* ADD register to register */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x1111); pc += 2;
        WW(pc, 0x4035); pc += 2;
        WW(pc, 0x2222); pc += 2;
        WW(pc, 0x5405); pc += 2;  /* ADD R4, R5 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("ADD R4+R5 = 0x3333", 0x3333, cpu.reg[5]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * DADD (BCD addition) tests — DADD #imm,R4 = 0xA034 (ADD opcode 5 -> A).
 * Regression net for the fix that replaced a plain binary add (no decimal
 * carry, no C flag) with real per-nibble BCD.
 * =================================================================== */
static void test_dadd(void) {
    printf("--- DADD (BCD) instruction tests ---\n");

    /* 9 DADD 1 = 0x10 (decimal 10), carry-in clear, carry-out clear */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4032); pc += 2; WW(pc, 0x0000); pc += 2; /* MOV #0, SR  (C=0) */
        WW(pc, 0x4034); pc += 2; WW(pc, 0x0009); pc += 2; /* MOV #0x09, R4 */
        WW(pc, 0xA034); pc += 2; WW(pc, 0x0001); pc += 2; /* DADD #0x01, R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("DADD 0x09 + 0x01 = 0x10", 0x10, cpu.reg[4] & 0xFFFF);
        assert_true("DADD 0x09+0x01 leaves C clear", (cpu.reg[MSP430_SR] & SR_C) == 0);
        cleanup_cpu(&cpu);
    }

    /* Byte-mode DADD.B 0x99 + 0x01 = 0x00 with decimal carry-out (99+1=100).
     * DADD.B #imm,R4 = 0xA074 (BW=1); byte op zeroes the upper register byte. */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4032); pc += 2; WW(pc, 0x0000); pc += 2; /* MOV #0, SR */
        WW(pc, 0x4034); pc += 2; WW(pc, 0x0099); pc += 2; /* MOV #0x99, R4 */
        WW(pc, 0xA074); pc += 2; WW(pc, 0x0001); pc += 2; /* DADD.B #0x01, R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("DADD.B 0x99 + 0x01 = 0x00", 0x00, cpu.reg[4] & 0xFFFF);
        assert_true("DADD.B 0x99+0x01 sets C", (cpu.reg[MSP430_SR] & SR_C) != 0);
        cleanup_cpu(&cpu);
    }

    /* Word DADD 0x99 + 0x01 = 0x0100 (no carry past 4 nibbles) */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4032); pc += 2; WW(pc, 0x0000); pc += 2; /* MOV #0, SR */
        WW(pc, 0x4034); pc += 2; WW(pc, 0x0099); pc += 2; /* MOV #0x99, R4 */
        WW(pc, 0xA034); pc += 2; WW(pc, 0x0001); pc += 2; /* DADD #0x01, R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("DADD 0x0099 + 0x0001 = 0x0100", 0x0100, cpu.reg[4] & 0xFFFF);
        assert_true("DADD 0x0099+0x0001 leaves C clear", (cpu.reg[MSP430_SR] & SR_C) == 0);
        cleanup_cpu(&cpu);
    }

    /* Word BCD carry ripple: 0x9999 DADD 0x0001 = 0x0000, C set */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4032); pc += 2; WW(pc, 0x0000); pc += 2; /* MOV #0, SR */
        WW(pc, 0x4034); pc += 2; WW(pc, 0x9999); pc += 2; /* MOV #0x9999, R4 */
        WW(pc, 0xA034); pc += 2; WW(pc, 0x0001); pc += 2; /* DADD #0x0001, R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("DADD 0x9999 + 0x0001 = 0x0000", 0x0000, cpu.reg[4] & 0xFFFF);
        assert_true("DADD 0x9999+0x0001 sets C", (cpu.reg[MSP430_SR] & SR_C) != 0);
        cleanup_cpu(&cpu);
    }

    /* Carry-in folds in: C=1, 0x09 DADD 0x00 = 0x10 */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4032); pc += 2; WW(pc, 0x0001); pc += 2; /* MOV #1, SR  (C=1) */
        WW(pc, 0x4034); pc += 2; WW(pc, 0x0009); pc += 2; /* MOV #0x09, R4 */
        WW(pc, 0xA034); pc += 2; WW(pc, 0x0000); pc += 2; /* DADD #0x00, R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("DADD 0x09 + 0 + carry = 0x10", 0x10, cpu.reg[4] & 0xFFFF);
        cleanup_cpu(&cpu);
    }

    /* Multi-nibble, no carries: 0x1234 DADD 0x1234 = 0x2468 */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4032); pc += 2; WW(pc, 0x0000); pc += 2; /* MOV #0, SR */
        WW(pc, 0x4034); pc += 2; WW(pc, 0x1234); pc += 2; /* MOV #0x1234, R4 */
        WW(pc, 0xA034); pc += 2; WW(pc, 0x1234); pc += 2; /* DADD #0x1234, R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("DADD 0x1234 + 0x1234 = 0x2468", 0x2468, cpu.reg[4] & 0xFFFF);
        assert_true("DADD 0x1234+0x1234 leaves C clear", (cpu.reg[MSP430_SR] & SR_C) == 0);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Logical operation tests
 * =================================================================== */
static void test_logical(void) {
    printf("--- Logical instruction tests ---\n");

    /* AND */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0xFF0F); pc += 2;
        WW(pc, 0xf034); pc += 2;  /* AND #0x0FF0, R4 */
        WW(pc, 0x0FF0); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("AND 0xFF0F & 0x0FF0 = 0x0F00", 0x0F00, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* BIS (OR) */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x00F0); pc += 2;
        WW(pc, 0xd034); pc += 2;  /* BIS #0x0F00, R4 */
        WW(pc, 0x0F00); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("BIS 0x00F0 | 0x0F00 = 0x0FF0", 0x0FF0, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* BIC (AND NOT) */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0xFFFF); pc += 2;
        WW(pc, 0xc034); pc += 2;  /* BIC #0x00FF, R4 */
        WW(pc, 0x00FF); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("BIC 0xFFFF & ~0x00FF = 0xFF00", 0xFF00, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* XOR */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0xAAAA); pc += 2;
        WW(pc, 0xe034); pc += 2;  /* XOR #0x5555, R4 */
        WW(pc, 0x5555); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("XOR 0xAAAA ^ 0x5555 = 0xFFFF", 0xFFFF, cpu.reg[4] & 0xFFFF);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Jump tests
 * =================================================================== */
static void test_jumps(void) {
    printf("--- Jump instruction tests ---\n");

    /* JMP unconditional */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x3c02); pc += 2;  /* JMP +2 (skip 4 bytes) */
        WW(pc, 0x4034); pc += 2;  /* MOV #0xDEAD, R4 (skipped) */
        WW(pc, 0xDEAD); pc += 2;
        WW(pc, 0x4034); pc += 2;  /* MOV #0xBEEF, R4 (target) */
        WW(pc, 0xBEEF); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);  /* JMP + MOV */
        assert_eq("JMP skips instruction", 0xBEEF, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* JEQ taken */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;  /* MOV #0, R4 */
        WW(pc, 0x4035); pc += 2;  /* MOV #0, R5 */
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x9405); pc += 2;  /* CMP R4, R5 */
        WW(pc, 0x2402); pc += 2;  /* JEQ +2 */
        WW(pc, 0x4036); pc += 2;  /* MOV #0xBAAD, R6 (skipped) */
        WW(pc, 0xBAAD); pc += 2;
        WW(pc, 0x4036); pc += 2;  /* MOV #0x600D, R6 (target) */
        WW(pc, 0x600D); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 5);
        assert_eq("JEQ taken when equal", 0x600D, cpu.reg[6]);
        cleanup_cpu(&cpu);
    }

    /* JNE not taken (Z=1) */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 5); pc += 2;
        WW(pc, 0x8034); pc += 2;  /* SUB #5, R4 => 0, Z=1 */
        WW(pc, 5); pc += 2;
        WW(pc, 0x2001); pc += 2;  /* JNE +1 */
        WW(pc, 0x4035); pc += 2;  /* MOV #0xAAAA, R5 */
        WW(pc, 0xAAAA); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 4);
        assert_eq("JNE not taken when Z=1", 0xAAAA, cpu.reg[5]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Byte mode tests
 * =================================================================== */
static void test_byte_mode(void) {
    printf("--- Byte mode tests ---\n");

    /* MOV.B #0xAA, R4 */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4074); pc += 2;
        WW(pc, 0x00AA); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);
        assert_eq("MOV.B #0xAA, R4", 0xAA, cpu.reg[4] & 0xFF);
        cleanup_cpu(&cpu);
    }

    /* ADD.B overflow */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4074); pc += 2;
        WW(pc, 0x00FF); pc += 2;
        WW(pc, 0x5354); pc += 2;  /* ADD.B #1, R4 (CG) */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("ADD.B 0xFF+1 wraps to 0", 0, cpu.reg[4] & 0xFF);
        assert_true("ADD.B overflow sets CARRY", (cpu.reg[MSP430_SR] & SR_C) != 0);
        cleanup_cpu(&cpu);
    }

    /* AND.B */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4074); pc += 2;
        WW(pc, 0x00FF); pc += 2;
        WW(pc, 0xf074); pc += 2;  /* AND.B #0x0F, R4 */
        WW(pc, 0x000F); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("AND.B 0xFF & 0x0F = 0x0F", 0x0F, cpu.reg[4] & 0xFF);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Single-operand instruction tests
 * =================================================================== */
static void test_single_operand(void) {
    printf("--- Single-operand instruction tests ---\n");

    /* SWPB */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x1234); pc += 2;
        WW(pc, 0x1084); pc += 2;  /* SWPB R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("SWPB 0x1234 => 0x3412", 0x3412, cpu.reg[4] & 0xFFFF);
        cleanup_cpu(&cpu);
    }

    /* SXT negative */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x0080); pc += 2;
        WW(pc, 0x1184); pc += 2;  /* SXT R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("SXT 0x0080 => 0xFF80", 0xFF80, cpu.reg[4] & 0xFFFF);
        cleanup_cpu(&cpu);
    }

    /* SXT positive */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x007F); pc += 2;
        WW(pc, 0x1184); pc += 2;  /* SXT R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("SXT 0x007F => 0x007F", 0x007F, cpu.reg[4] & 0xFFFF);
        cleanup_cpu(&cpu);
    }

    /* RRA negative (sign preserved) */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x8004); pc += 2;
        WW(pc, 0x1104); pc += 2;  /* RRA R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("RRA 0x8004 => 0xC002", 0xC002, cpu.reg[4] & 0xFFFF);
        cleanup_cpu(&cpu);
    }

    /* RRA positive */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x0008); pc += 2;
        WW(pc, 0x1104); pc += 2;  /* RRA R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("RRA 0x0008 => 0x0004", 0x0004, cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* PUSH/POP */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        int stack_top = 0x1100 + 0x200;
        WW(pc, 0x4031); pc += 2;
        WW(pc, stack_top); pc += 2;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0xCAFE); pc += 2;
        WW(pc, 0x1204); pc += 2;  /* PUSH R4 */
        WW(pc, 0x4304); pc += 2;  /* MOV #0, R4 */
        WW(pc, 0x4134); pc += 2;  /* MOV @SP+, R4 (POP) */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 5);
        assert_eq("PUSH/POP R4 preserves 0xCAFE", 0xCAFE, cpu.reg[4] & 0xFFFF);
        assert_eq("SP restored after PUSH/POP", (long)stack_top, (long)cpu.reg[MSP430_SP]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Cycle count tests
 * =================================================================== */
static void test_cycle_counts(void) {
    printf("--- Cycle count tests ---\n");

    /* MOV Rn, Rm: 1 cycle */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;  /* dummy: MOV #0, R4 */
        WW(pc, 0x4405); pc += 2;  /* MOV R4, R5 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);  /* absorb reset + dummy */
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("MOV Rn,Rm = 1 cycle", 1, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* MOV #imm, Rd: 2 cycles */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x1234); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("MOV #imm,Rd = 2 cycles", 2, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* JMP: 2 cycles */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;
        WW(pc, 0x3fff); pc += 2;  /* JMP $ */
        msp430_step(&cpu, 1);
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("JMP = 2 cycles", 2, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* ADD #1 (CG): 1 cycle */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;
        WW(pc, 0x5314); pc += 2;  /* ADD #1, R4 (CG) */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("ADD CG,Rd = 1 cycle", 1, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* ADD #imm, Rd: 2 cycles */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;
        WW(pc, 0x5034); pc += 2;  /* ADD #imm, R4 */
        WW(pc, 0x0064); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("ADD #imm,Rd = 2 cycles", 2, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* PUSH Rn: 3 cycles */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        int stack_top = 0x1100 + 0x200;
        WW(pc, 0x4031); pc += 2;
        WW(pc, stack_top); pc += 2;
        WW(pc, 0x1204); pc += 2;  /* PUSH R4 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);  /* absorb reset + MOV SP */
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("PUSH Rn = 3 cycles", 3, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* CALL #addr: 5 cycles */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        int stack_top = 0x1100 + 0x200;
        WW(pc, 0x4031); pc += 2;
        WW(pc, stack_top); pc += 2;
        int call_addr = 0x4100;
        WW(pc, 0x12b0); pc += 2;  /* CALL #addr */
        WW(pc, call_addr); pc += 2;
        WW(call_addr, 0x3fff);
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);  /* absorb reset + MOV SP */
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 1);
        assert_eq("CALL #addr = 5 cycles", 5, cpu.cycles - c0);
        cleanup_cpu(&cpu);
    }

    /* Cumulative cycle count for a known loop */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4304); pc += 2;  /* dummy: MOV #0, R4 */
        WW(pc, 0x4035); pc += 2;  /* MOV #10, R5 */
        WW(pc, 10); pc += 2;
        int loop_start = pc;
        WW(pc, 0x5314); pc += 2;  /* ADD #1, R4 */
        WW(pc, 0x8315); pc += 2;  /* SUB #1, R5 */
        int offset = (loop_start - (pc + 2)) / 2;
        WW(pc, 0x2000 | (offset & 0x3ff)); pc += 2;  /* JNE loop */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);  /* absorb reset + dummy */
        int64_t c0 = cpu.cycles;
        msp430_step(&cpu, 31);  /* MOV #10 + 10*(ADD+SUB+JNE) */
        assert_eq("Loop 10 iters: 2 + 10*(1+1+2) = 42 cycles", 42, cpu.cycles - c0);
        assert_eq("R4 after 10 iters", 10, cpu.reg[4]);
        assert_eq("R5 after 10 iters", 0, cpu.reg[5]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Memory operation tests
 * =================================================================== */
static void test_memory_ops(void) {
    printf("--- Memory operation tests ---\n");

    /* Write and read back from RAM */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;  /* MOV #RAM_START, R4 */
        WW(pc, 0x1100); pc += 2;
        WW(pc, 0x40b4); pc += 2;  /* MOV #0x1234, 0(R4) */
        WW(pc, 0x1234); pc += 2;
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x4415); pc += 2;  /* MOV 0(R4), R5 */
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_eq("Write/read RAM: 0x1234", 0x1234, cpu.reg[5]);
        cleanup_cpu(&cpu);
    }

    /* Indirect auto-increment @R4+ */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        WW(0x1100, 0x1111);
        WW(0x1102, 0x2222);
        WW(0x1104, 0x3333);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;  /* MOV #RAM_START, R4 */
        WW(pc, 0x1100); pc += 2;
        WW(pc, 0x4435); pc += 2;  /* MOV @R4+, R5 */
        WW(pc, 0x4436); pc += 2;  /* MOV @R4+, R6 */
        WW(pc, 0x4437); pc += 2;  /* MOV @R4+, R7 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 4);
        assert_eq("@R4+ first word", 0x1111, cpu.reg[5]);
        assert_eq("@R4+ second word", 0x2222, cpu.reg[6]);
        assert_eq("@R4+ third word", 0x3333, cpu.reg[7]);
        assert_eq("R4 advanced by 6", 0x1100 + 6, (long)cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* Byte write/read to RAM */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 0x1100); pc += 2;
        WW(pc, 0x40f4); pc += 2;  /* MOV.B #0xAB, 0(R4) */
        WW(pc, 0x00AB); pc += 2;
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("MOV.B #0xAB to RAM", 0xAB, cpu.memory[0x1100] & 0xFF);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * CALL/RET tests
 * =================================================================== */
static void test_call_return(void) {
    printf("--- CALL/RET tests ---\n");

    /* Basic CALL and RET */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int stack_top = 0x1100 + 0x200;
        int func_addr = 0x4080;
        int pc = 0x4000;

        WW(pc, 0x4031); pc += 2;
        WW(pc, stack_top); pc += 2;
        WW(pc, 0x4304); pc += 2;  /* MOV #0, R4 */
        WW(pc, 0x12b0); pc += 2;  /* CALL #func */
        WW(pc, func_addr); pc += 2;
        int after_call = pc;
        WW(pc, 0x3fff);

        pc = func_addr;
        WW(pc, 0x4034); pc += 2;  /* MOV #42, R4 */
        WW(pc, 42); pc += 2;
        WW(pc, 0x4130);           /* RET */

        msp430_step(&cpu, 5);
        assert_eq("CALL/RET: R4 = 42", 42, cpu.reg[4]);
        assert_eq("CALL/RET: PC after return", after_call, (long)cpu.reg[MSP430_PC]);
        assert_eq("CALL/RET: SP restored", (long)stack_top, (long)cpu.reg[MSP430_SP]);
        cleanup_cpu(&cpu);
    }

    /* Nested CALL */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int stack_top = 0x1100 + 0x200;
        int func1 = 0x4080;
        int func2 = 0x4100;
        int pc = 0x4000;

        WW(pc, 0x4031); pc += 2;
        WW(pc, stack_top); pc += 2;
        WW(pc, 0x4304); pc += 2;  /* MOV #0, R4 */
        WW(pc, 0x12b0); pc += 2;  /* CALL #func1 */
        WW(pc, func1); pc += 2;
        WW(pc, 0x3fff);

        pc = func1;
        WW(pc, 0x5034); pc += 2;  /* ADD #10, R4 */
        WW(pc, 10); pc += 2;
        WW(pc, 0x12b0); pc += 2;  /* CALL #func2 */
        WW(pc, func2); pc += 2;
        WW(pc, 0x4130);           /* RET */

        pc = func2;
        WW(pc, 0x5034); pc += 2;  /* ADD #20, R4 */
        WW(pc, 20); pc += 2;
        WW(pc, 0x4130);           /* RET */

        msp430_step(&cpu, 8);
        assert_eq("Nested CALL: R4 = 30", 30, cpu.reg[4]);
        assert_eq("Nested CALL: SP restored", (long)stack_top, (long)cpu.reg[MSP430_SP]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * CMP and flag tests
 * =================================================================== */
static void test_cmp_flags(void) {
    printf("--- CMP and flag tests ---\n");

    /* CMP equal: Z=1, C=1 */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 5); pc += 2;
        WW(pc, 0x4035); pc += 2;
        WW(pc, 5); pc += 2;
        WW(pc, 0x9405); pc += 2;  /* CMP R4, R5 */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_true("CMP equal: Z=1", (cpu.reg[MSP430_SR] & SR_Z) != 0);
        assert_true("CMP equal: C=1", (cpu.reg[MSP430_SR] & SR_C) != 0);
        cleanup_cpu(&cpu);
    }

    /* CMP R5 > R4: Z=0, C=1, N=0 */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 3); pc += 2;
        WW(pc, 0x4035); pc += 2;
        WW(pc, 10); pc += 2;
        WW(pc, 0x9405); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_true("CMP R5>R4: Z=0", (cpu.reg[MSP430_SR] & SR_Z) == 0);
        assert_true("CMP R5>R4: C=1", (cpu.reg[MSP430_SR] & SR_C) != 0);
        assert_true("CMP R5>R4: N=0", (cpu.reg[MSP430_SR] & SR_N) == 0);
        cleanup_cpu(&cpu);
    }

    /* CMP R5 < R4: C=0, N=1 */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f1611_config, 0x4000);
        int pc = 0x4000;
        WW(pc, 0x4034); pc += 2;
        WW(pc, 10); pc += 2;
        WW(pc, 0x4035); pc += 2;
        WW(pc, 3); pc += 2;
        WW(pc, 0x9405); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 3);
        assert_true("CMP R5<R4: C=0", (cpu.reg[MSP430_SR] & SR_C) == 0);
        assert_true("CMP R5<R4: N=1", (cpu.reg[MSP430_SR] & SR_N) != 0);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * MSP430X tests
 * =================================================================== */
static void test_msp430x(void) {
    printf("--- MSP430X instruction tests ---\n");

    /* MOVA #imm, Rd */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f5437_config, 0x5c00);
        int pc = 0x5c00;
        WW(pc, 0x0184); pc += 2;  /* MOVA #0x12345, R4 (high=1) */
        WW(pc, 0x2345); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);
        assert_eq("MOVA #0x12345,R4", 0x12345, (long)cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* ADDA #imm, Rd */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f5437_config, 0x5c00);
        int pc = 0x5c00;
        WW(pc, 0x0184); pc += 2;  /* MOVA #0x10000, R4 */
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x00a4); pc += 2;  /* ADDA #5, R4 */
        WW(pc, 0x0005); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("ADDA 0x10000+5 = 0x10005", 0x10005, (long)cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* SUBA #imm, Rd */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f5437_config, 0x5c00);
        int pc = 0x5c00;
        WW(pc, 0x0184); pc += 2;  /* MOVA #0x10000, R4 */
        WW(pc, 0x0000); pc += 2;
        WW(pc, 0x00b4); pc += 2;  /* SUBA #1, R4 */
        WW(pc, 0x0001); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_eq("SUBA 0x10000-1 = 0x0FFFF", 0x0FFFF, (long)cpu.reg[4]);
        cleanup_cpu(&cpu);
    }

    /* CMPA equal */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f5437_config, 0x5c00);
        int pc = 0x5c00;
        WW(pc, 0x0184); pc += 2;  /* MOVA #0x12345, R4 */
        WW(pc, 0x2345); pc += 2;
        WW(pc, 0x0194); pc += 2;  /* CMPA #0x12345, R4 */
        WW(pc, 0x2345); pc += 2;
        WW(pc, 0x3fff);
        msp430_step(&cpu, 2);
        assert_true("CMPA equal: Z=1", (cpu.reg[MSP430_SR] & SR_Z) != 0);
        cleanup_cpu(&cpu);
    }

    /* PUSHX.A #imm + RETA round-trip.
     *
     * Reproduces the Z1 RPL-UDP boot bug: PUSHX.A only adjusted SP by 2
     * (writing 16 bits) instead of 4 (writing 20 bits), so the next reta
     * popped 2 bytes of pushed value plus 2 bytes of unrelated stack as
     * the return-address word. Verify here that SP moves by 4 and that a
     * RETA round-trip restores the expected PC.
     *
     * Program: pushx.a #0x12345 ; reta
     * Layout in flash (little-endian words):
     *   0x5c00: 0x1880  ext word (A/L=0 -> .A mode, src/dst hi nibbles 0)
     *   0x5c02: 0x1230  push.b @r0+ (PUSH with immediate via PC autoinc;
     *                                 BW bit is set but the ext word .A
     *                                 mode forces 20-bit operation)
     *   0x5c04: 0x2345  low 16 bits of immediate
     *   0x5c06: 0x0110  RETA (= MOVA @SP+, PC)
     */
    {
        msp430_cpu_t cpu;
        setup_cpu(&cpu, &msp430f5437_config, 0x5c00);
        cpu.reg[MSP430_SP] = 0x2000;
        int pc = 0x5c00;
        WW(pc, 0x1801); pc += 2;  /* ext word: A/L=0 (.A), dst_hi=1 -> imm = 0x12345 */
        WW(pc, 0x1230); pc += 2;  /* push #imm (PC autoinc, .A via ext) */
        WW(pc, 0x2345); pc += 2;  /* low 16 bits */
        WW(pc, 0x0110); pc += 2;  /* RETA */
        WW(pc, 0x3fff);
        msp430_step(&cpu, 1);
        assert_eq("PUSHX.A SP -= 4", 0x1FFC, (long)cpu.reg[MSP430_SP]);
        assert_eq("PUSHX.A wrote 20-bit immediate",
                  0x12345,
                  (long)(msp430_read_word(&cpu, 0x1FFC) |
                         ((msp430_read_word(&cpu, 0x1FFE) & 0xf) << 16)));
        msp430_step(&cpu, 1);
        assert_eq("RETA after PUSHX.A jumps to 0x12345", 0x12345, (long)cpu.reg[MSP430_PC]);
        assert_eq("RETA after PUSHX.A SP back to 0x2000", 0x2000, (long)cpu.reg[MSP430_SP]);
        cleanup_cpu(&cpu);
    }
}

/* ===================================================================
 * Run all correctness tests
 * =================================================================== */
int run_correctness_tests(int v) {
    passed = 0;
    failed = 0;
    verbose = v;

    printf("=== MSP430 C Emulator Correctness Tests ===\n\n");

    test_mov();
    test_arithmetic();
    test_dadd();
    test_logical();
    test_jumps();
    test_byte_mode();
    test_single_operand();
    test_cycle_counts();
    test_memory_ops();
    test_call_return();
    test_cmp_flags();
    test_msp430x();

    printf("\n=== Summary ===\n");
    printf("Passed: %d\n", passed);
    printf("Failed: %d\n", failed);
    printf("Total:  %d\n", passed + failed);
    printf("\nRESULT: %s\n", failed > 0 ? "FAIL" : "PASS");

    return failed;
}
