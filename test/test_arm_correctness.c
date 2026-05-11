/*
 * Correctness tests for the ARM Cortex-M3 CPU emulator.
 */
#include "arm_cpu.h"
#include "arm_config.h"
#include <stdio.h>
#include <string.h>

static int passed = 0;
static int failed = 0;
static int verbose = 0;

static void assert_eq(const char *name, uint32_t expected, uint32_t actual) {
    if (expected == actual) {
        passed++;
        if (verbose) printf("  PASS: %s\n", name);
    } else {
        failed++;
        printf("  FAIL: %s (expected 0x%x, got 0x%x)\n", name, expected, actual);
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

/* Helper: set up CPU with code at flash base */
static void setup_arm(arm_cpu_t *cpu) {
    arm_cpu_init(cpu, &cc2538_config);
    /* Set initial SP */
    arm_write32(cpu, ARM_FLASH_BASE, ARM_SRAM_BASE + ARM_SRAM_SIZE);
    /* Set initial PC (entry point) - with thumb bit */
    arm_write32(cpu, ARM_FLASH_BASE + 4, ARM_FLASH_BASE + 0x100 + 1);
    arm_cpu_reset(cpu);
}

/* Write 16-bit Thumb instruction into flash */
static void write_thumb16(arm_cpu_t *cpu, uint32_t addr, uint16_t insn) {
    uint32_t off = addr - ARM_FLASH_BASE;
    cpu->flash[off] = insn & 0xFF;
    cpu->flash[off + 1] = (insn >> 8) & 0xFF;
}

/* Write 32-bit Thumb-2 instruction into flash */
static void write_thumb32(arm_cpu_t *cpu, uint32_t addr, uint16_t hw1, uint16_t hw2) {
    write_thumb16(cpu, addr, hw1);
    write_thumb16(cpu, addr + 2, hw2);
}

#define CODE_BASE (ARM_FLASH_BASE + 0x100)

/* ===================================================================
 * MOV instruction tests
 * =================================================================== */
static void test_mov(void) {
    printf("--- MOV instruction tests ---\n");

    /* MOV Rd, #imm8 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2042); pc += 2; /* MOVS R0, #0x42 */
        write_thumb16(&cpu, pc, 0x21FF); pc += 2; /* MOVS R1, #0xFF */
        arm_step(&cpu, 2);
        assert_eq("MOVS R0, #0x42", 0x42, cpu.reg[0]);
        assert_eq("MOVS R1, #0xFF", 0xFF, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* MOV Rd, Rm (high register) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2055); pc += 2; /* MOVS R0, #0x55 */
        write_thumb16(&cpu, pc, 0x4680); pc += 2; /* MOV R8, R0 */
        arm_step(&cpu, 2);
        assert_eq("MOV R8, R0", 0x55, cpu.reg[8]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * ADD / SUB instruction tests
 * =================================================================== */
static void test_add_sub(void) {
    printf("--- ADD/SUB instruction tests ---\n");

    /* ADD Rd, Rn, Rm */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x200A); pc += 2; /* MOVS R0, #10 */
        write_thumb16(&cpu, pc, 0x2114); pc += 2; /* MOVS R1, #20 */
        write_thumb16(&cpu, pc, 0x1842); pc += 2; /* ADDS R2, R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("ADDS R2, R0, R1 = 30", 30, cpu.reg[2]);
        assert_true("Z flag clear", !(cpu.xpsr & APSR_Z));
        arm_cpu_destroy(&cpu);
    }

    /* SUB Rd, #imm8 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2064); pc += 2; /* MOVS R0, #100 */
        write_thumb16(&cpu, pc, 0x3819); pc += 2; /* SUBS R0, #25 */
        arm_step(&cpu, 2);
        assert_eq("SUBS R0, #25 = 75", 75, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* SUB that produces zero */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2005); pc += 2; /* MOVS R0, #5 */
        write_thumb16(&cpu, pc, 0x3805); pc += 2; /* SUBS R0, #5 */
        arm_step(&cpu, 2);
        assert_eq("SUBS R0, #5 = 0", 0, cpu.reg[0]);
        assert_true("Z flag set", (cpu.xpsr & APSR_Z) != 0);
        assert_true("C flag set (no borrow)", (cpu.xpsr & APSR_C) != 0);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * CMP instruction tests
 * =================================================================== */
static void test_cmp(void) {
    printf("--- CMP instruction tests ---\n");

    /* CMP equal */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2042); pc += 2; /* MOVS R0, #0x42 */
        write_thumb16(&cpu, pc, 0x2842); pc += 2; /* CMP R0, #0x42 */
        arm_step(&cpu, 2);
        assert_true("CMP equal: Z set", (cpu.xpsr & APSR_Z) != 0);
        assert_true("CMP equal: C set", (cpu.xpsr & APSR_C) != 0);
        arm_cpu_destroy(&cpu);
    }

    /* CMP greater */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2050); pc += 2; /* MOVS R0, #0x50 */
        write_thumb16(&cpu, pc, 0x2810); pc += 2; /* CMP R0, #0x10 */
        arm_step(&cpu, 2);
        assert_true("CMP greater: Z clear", !(cpu.xpsr & APSR_Z));
        assert_true("CMP greater: C set", (cpu.xpsr & APSR_C) != 0);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Logic instruction tests
 * =================================================================== */
static void test_logic(void) {
    printf("--- Logic instruction tests ---\n");

    /* AND */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x20FF); pc += 2; /* MOVS R0, #0xFF */
        write_thumb16(&cpu, pc, 0x210F); pc += 2; /* MOVS R1, #0x0F */
        write_thumb16(&cpu, pc, 0x4008); pc += 2; /* ANDS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("ANDS R0, R1 = 0x0F", 0x0F, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* ORR */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x20F0); pc += 2; /* MOVS R0, #0xF0 */
        write_thumb16(&cpu, pc, 0x210F); pc += 2; /* MOVS R1, #0x0F */
        write_thumb16(&cpu, pc, 0x4308); pc += 2; /* ORRS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("ORRS R0, R1 = 0xFF", 0xFF, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* EOR */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x20FF); pc += 2; /* MOVS R0, #0xFF */
        write_thumb16(&cpu, pc, 0x210F); pc += 2; /* MOVS R1, #0x0F */
        write_thumb16(&cpu, pc, 0x4048); pc += 2; /* EORS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("EORS R0, R1 = 0xF0", 0xF0, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* BIC */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x20FF); pc += 2; /* MOVS R0, #0xFF */
        write_thumb16(&cpu, pc, 0x210F); pc += 2; /* MOVS R1, #0x0F */
        write_thumb16(&cpu, pc, 0x4388); pc += 2; /* BICS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("BICS R0, R1 = 0xF0", 0xF0, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* MVN */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2000); pc += 2; /* MOVS R0, #0 */
        write_thumb16(&cpu, pc, 0x43C0); pc += 2; /* MVNS R0, R0 */
        arm_step(&cpu, 2);
        assert_eq("MVNS R0, R0 = 0xFFFFFFFF", 0xFFFFFFFF, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Shift instruction tests
 * =================================================================== */
static void test_shifts(void) {
    printf("--- Shift instruction tests ---\n");

    /* LSL immediate */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2001); pc += 2; /* MOVS R0, #1 */
        write_thumb16(&cpu, pc, 0x0200); pc += 2; /* LSLS R0, R0, #8 */
        arm_step(&cpu, 2);
        assert_eq("LSLS R0, R0, #8 = 0x100", 0x100, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* LSR immediate */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x20FF); pc += 2; /* MOVS R0, #0xFF */
        write_thumb16(&cpu, pc, 0x0A01); pc += 2; /* LSRS R1, R0, #8 */
        arm_step(&cpu, 2);
        assert_eq("LSRS R1, R0, #8 = 0", 0, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* MUL */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2007); pc += 2; /* MOVS R0, #7 */
        write_thumb16(&cpu, pc, 0x2106); pc += 2; /* MOVS R1, #6 */
        write_thumb16(&cpu, pc, 0x4348); pc += 2; /* MULS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("MULS R0, R1 = 42", 42, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Load/Store tests
 * =================================================================== */
static void test_load_store(void) {
    printf("--- Load/Store instruction tests ---\n");

    /* STR/LDR with immediate offset */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        /* MOVS R0, #0xAB */
        write_thumb16(&cpu, pc, 0x20AB); pc += 2;
        /* Load SP-relative address into R1 */
        uint32_t sram_addr = ARM_SRAM_BASE + 0x100;
        /* MOVW R1, #lower16(sram_addr) */
        uint16_t imm16 = sram_addr & 0xFFFF;
        uint16_t hw1 = 0xF240 | ((imm16 >> 12) & 0xF) | (((imm16 >> 11) & 1) << 10);
        uint16_t hw2 = ((imm16 >> 8) & 7) << 12 | (1 << 8) | (imm16 & 0xFF);
        write_thumb32(&cpu, pc, hw1, hw2); pc += 4;
        /* MOVT R1, #upper16(sram_addr) */
        uint16_t hi16 = (sram_addr >> 16) & 0xFFFF;
        uint16_t hw1t = 0xF2C0 | ((hi16 >> 12) & 0xF) | (((hi16 >> 11) & 1) << 10);
        uint16_t hw2t = ((hi16 >> 8) & 7) << 12 | (1 << 8) | (hi16 & 0xFF);
        write_thumb32(&cpu, pc, hw1t, hw2t); pc += 4;
        /* STR R0, [R1, #0] */
        write_thumb16(&cpu, pc, 0x6008); pc += 2;
        /* MOVS R0, #0 */
        write_thumb16(&cpu, pc, 0x2000); pc += 2;
        /* LDR R2, [R1, #0] */
        write_thumb16(&cpu, pc, 0x680A); pc += 2;
        arm_step(&cpu, 6);
        assert_eq("STR/LDR: R2 = 0xAB", 0xAB, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* PUSH/POP */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2042); pc += 2; /* MOVS R0, #0x42 */
        write_thumb16(&cpu, pc, 0x2155); pc += 2; /* MOVS R1, #0x55 */
        write_thumb16(&cpu, pc, 0xB503); pc += 2; /* PUSH {R0, R1, LR} */
        write_thumb16(&cpu, pc, 0x2000); pc += 2; /* MOVS R0, #0 */
        write_thumb16(&cpu, pc, 0x2100); pc += 2; /* MOVS R1, #0 */
        write_thumb16(&cpu, pc, 0xBD03); pc += 2; /* POP {R0, R1, PC} */
        uint32_t sp_before = cpu.reg[ARM_SP];
        arm_step(&cpu, 4); /* MOVS, MOVS, PUSH, MOVS */
        assert_eq("After PUSH: SP decreased", sp_before - 12, cpu.reg[ARM_SP]);
        arm_step(&cpu, 1); /* MOVS R1 */
        arm_step(&cpu, 1); /* POP */
        assert_eq("POP R0 = 0x42", 0x42, cpu.reg[0]);
        assert_eq("POP R1 = 0x55", 0x55, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Branch tests
 * =================================================================== */
static void test_branch(void) {
    printf("--- Branch instruction tests ---\n");

    /* Conditional branch (BEQ) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2000); pc += 2; /* MOVS R0, #0 */
        write_thumb16(&cpu, pc, 0x2800); pc += 2; /* CMP R0, #0 */
        write_thumb16(&cpu, pc, 0xD001); pc += 2; /* BEQ +2 (skip 2 insns) */
        write_thumb16(&cpu, pc, 0x2101); pc += 2; /* MOVS R1, #1 (skipped) */
        write_thumb16(&cpu, pc, 0x2201); pc += 2; /* MOVS R2, #1 (skipped) */
        write_thumb16(&cpu, pc, 0x2301); pc += 2; /* MOVS R3, #1 (target) */
        arm_step(&cpu, 4);
        assert_eq("BEQ taken: R1 unchanged", 0, cpu.reg[1]);
        assert_eq("BEQ target: R3 = 1", 1, cpu.reg[3]);
        arm_cpu_destroy(&cpu);
    }

    /* BX LR (return) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        uint32_t ret_addr = CODE_BASE + 0x10;
        /* Set LR directly from test harness (with Thumb bit) */
        cpu.reg[ARM_LR] = ret_addr | 1;
        /* BX LR */
        write_thumb16(&cpu, pc, 0x4770); pc += 2;
        /* At ret_addr: MOVS R5, #0x99 */
        write_thumb16(&cpu, ret_addr, 0x2599);
        arm_step(&cpu, 2); /* BX LR, then MOVS R5 */
        assert_eq("BX LR: R5 = 0x99", 0x99, cpu.reg[5]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Extension instruction tests
 * =================================================================== */
static void test_extensions(void) {
    printf("--- Extension instruction tests ---\n");

    /* UXTB */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        /* MOVS R0, #0xFF wouldn't set upper bits; use SUB to get negative */
        write_thumb16(&cpu, pc, 0x2000); pc += 2; /* MOVS R0, #0 */
        write_thumb16(&cpu, pc, 0x3801); pc += 2; /* SUBS R0, #1 => 0xFFFFFFFF */
        write_thumb16(&cpu, pc, 0xB2C1); pc += 2; /* UXTB R1, R0 */
        arm_step(&cpu, 3);
        assert_eq("UXTB R1, R0 = 0xFF", 0xFF, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* SXTB */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2080); pc += 2; /* MOVS R0, #0x80 */
        write_thumb16(&cpu, pc, 0xB241); pc += 2; /* SXTB R1, R0 */
        arm_step(&cpu, 2);
        assert_eq("SXTB R1, R0 = 0xFFFFFF80", 0xFFFFFF80, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* UXTH */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2000); pc += 2; /* MOVS R0, #0 */
        write_thumb16(&cpu, pc, 0x3801); pc += 2; /* SUBS R0, #1 => 0xFFFFFFFF */
        write_thumb16(&cpu, pc, 0xB281); pc += 2; /* UXTH R1, R0 */
        arm_step(&cpu, 3);
        assert_eq("UXTH R1, R0 = 0xFFFF", 0xFFFF, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * ADC / SBC / RSB tests
 * =================================================================== */
static void test_adc_sbc(void) {
    printf("--- ADC/SBC/RSB instruction tests ---\n");

    /* RSB (NEG) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2005); pc += 2; /* MOVS R0, #5 */
        write_thumb16(&cpu, pc, 0x4240); pc += 2; /* RSBS R0, R0, #0 (NEG) */
        arm_step(&cpu, 2);
        assert_eq("RSBS R0 (NEG 5) = -5", (uint32_t)-5, cpu.reg[0]);
        assert_true("NEG sets N flag", (cpu.xpsr & APSR_N) != 0);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Cortex-M4 DSP extension — halfword multiply family
 *
 * Encoding reference: ARMv7-M Architecture Reference Manual, A7.7.121–
 * A7.7.169 (SMLA/SMUL halfword variants), A7.7.123 (SMLAL halfword).
 *
 * Common encoding sketch:
 *   SMUL{B,T}{B,T}: 1111 1011 0001 Rn | 1111 Rd  00 N M Rm   (hw1=0xFB1n)
 *   SMLA{B,T}{B,T}: 1111 1011 0001 Rn | Ra   Rd  00 N M Rm
 *   SMULW{B,T}    : 1111 1011 0011 Rn | 1111 Rd  000 M Rm    (hw1=0xFB3n)
 *   SMLAW{B,T}    : 1111 1011 0011 Rn | Ra   Rd  000 M Rm
 *   SMLAL{B,T}{B,T}: 1111 1011 1100 Rn | RdLo RdHi 10 N M Rm (hw1=0xFBCn)
 *
 * N selects top (1) vs bottom (0) half of Rn; M selects top vs bottom of Rm.
 * =================================================================== */
static void test_m4_dsp_halfword_multiply(void) {
    printf("--- M4 DSP halfword multiply tests ---\n");

    /* SMULBB R2, R0, R1 — R0[15:0] * R1[15:0] (signed) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0xDEAD0007;   /* low half = 7 */
        cpu.reg[1] = 0xBEEF000A;   /* low half = 10 */
        /* Encoding: hw1 = 0xFB10 | Rn(0), hw2 = 0xF000 | Rd(2)<<8 | 0x00 | Rm(1) */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0xF201);
        arm_step(&cpu, 1);
        assert_eq("SMULBB R2 = 7*10", 70, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMULBT R2, R0, R1 — R0[15:0] * R1[31:16] */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00000003;
        cpu.reg[1] = 0x00040000;   /* top half = 4 */
        /* op2_misc = 01 (M=1, N=0): hw2 = 0xF000 | Rd<<8 | 0x10 | Rm */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0xF211);
        arm_step(&cpu, 1);
        assert_eq("SMULBT R2 = 3 * top(4)", 12, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMULTB R2, R0, R1 — R0[31:16] * R1[15:0] */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00050000;
        cpu.reg[1] = 0x00000006;
        /* op2_misc = 10 (N=1, M=0): hw2 = 0xF000 | Rd<<8 | 0x20 | Rm */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0xF221);
        arm_step(&cpu, 1);
        assert_eq("SMULTB R2 = top(5) * 6", 30, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMULTT R2, R0, R1 — R0[31:16] * R1[31:16] */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00070000;
        cpu.reg[1] = 0x00080000;
        /* op2_misc = 11 (N=1, M=1): hw2 = 0xF000 | Rd<<8 | 0x30 | Rm */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0xF231);
        arm_step(&cpu, 1);
        assert_eq("SMULTT R2 = top(7) * top(8)", 56, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMULBB with negative inputs — checks signed sign-extension */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x0000FFFF;   /* low half = -1 (signed 16) */
        cpu.reg[1] = 0x00000003;   /* low half = 3 */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0xF201);
        arm_step(&cpu, 1);
        assert_eq("SMULBB R2 = -1 * 3", (uint32_t)-3, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMLABB R2, R0, R1, R3 — R3 + R0[15:0] * R1[15:0] */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00000007;
        cpu.reg[1] = 0x0000000A;
        cpu.reg[3] = 100;
        /* hw2 = Ra(3)<<12 | Rd(2)<<8 | 0x00 | Rm(1) */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0x3201);
        arm_step(&cpu, 1);
        assert_eq("SMLABB R2 = 100 + 7*10", 170, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMLATT R2, R0, R1, R3 with overflow → Q flag set, low 32 bits stored */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        /* top(R0)=0x7FFF (max int16), top(R1)=0x7FFF — product = 0x3FFF0001 */
        cpu.reg[0] = 0x7FFF0000;
        cpu.reg[1] = 0x7FFF0000;
        cpu.reg[3] = 0x7FFFFFFF;   /* near max int32 */
        write_thumb32(&cpu, CODE_BASE, 0xFB10, 0x3231);
        arm_step(&cpu, 1);
        /* Sum = 0x7FFFFFFF + 0x3FFF0001 overflows int32 → Q sticky */
        assert_eq("SMLATT R2 = wrapped sum",
                  (uint32_t)(0x7FFFFFFF + 0x3FFF0001), cpu.reg[2]);
        assert_true("SMLATT overflow sets Q", (cpu.xpsr & APSR_Q) != 0);
        arm_cpu_destroy(&cpu);
    }

    /* SMULWB R2, R0, R1 — (R0:32 * R1[15:0]) >> 16, taking middle 32 bits */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00010000;   /* 65536 */
        cpu.reg[1] = 0x00000004;   /* low half = 4 */
        /* hw1 = 0xFB30 | Rn(0); op2_misc = 0 (M=0): hw2 = 0xF000 | Rd<<8 | 0x00 | Rm */
        write_thumb32(&cpu, CODE_BASE, 0xFB30, 0xF201);
        arm_step(&cpu, 1);
        /* 65536 * 4 = 262144 = 0x40000; >> 16 = 4 */
        assert_eq("SMULWB R2 = (65536 * 4) >> 16", 4, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMULWT R2, R0, R1 — (R0:32 * R1[31:16]) >> 16 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00010000;
        cpu.reg[1] = 0x00050000;   /* top half = 5 */
        /* op2_misc = 1 (M=1): hw2 = 0xF000 | Rd<<8 | 0x10 | Rm */
        write_thumb32(&cpu, CODE_BASE, 0xFB30, 0xF211);
        arm_step(&cpu, 1);
        assert_eq("SMULWT R2 = (65536 * 5) >> 16", 5, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMLAWB R2, R0, R1, R3 — R3 + ((R0 * R1[15:0]) >> 16) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00020000;
        cpu.reg[1] = 0x00000003;
        cpu.reg[3] = 1000;
        /* hw2 = Ra(3)<<12 | Rd(2)<<8 | 0x00 | Rm(1) */
        write_thumb32(&cpu, CODE_BASE, 0xFB30, 0x3201);
        arm_step(&cpu, 1);
        /* (131072 * 3) >> 16 = 393216 >> 16 = 6; + 1000 = 1006 */
        assert_eq("SMLAWB R2 = 1000 + ((131072*3)>>16)", 1006, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* SMLALBB R3:R2 += R0[15:0] * R1[15:0] (signed)
       hw1 = 0xFBC0 | Rn(0); hw2 = RdLo(2)<<12 | RdHi(3)<<8 | 0x80 | Rm(1) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x000000C8;   /* 200 */
        cpu.reg[1] = 0x00000005;   /* 5 */
        cpu.reg[2] = 50;            /* RdLo (acc low)  */
        cpu.reg[3] = 0;             /* RdHi (acc high) */
        write_thumb32(&cpu, CODE_BASE, 0xFBC0, 0x2381);
        arm_step(&cpu, 1);
        /* Acc starts at 50, += 200*5=1000 → 1050 */
        assert_eq("SMLALBB acc lo = 1050", 1050, cpu.reg[2]);
        assert_eq("SMLALBB acc hi = 0", 0, cpu.reg[3]);
        arm_cpu_destroy(&cpu);
    }

    /* SMLALTT — accumulator carries into high word */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x70000000;   /* top = 0x7000 */
        cpu.reg[1] = 0x40000000;   /* top = 0x4000 */
        cpu.reg[2] = 0xFFFFFFFF;   /* RdLo near wrap */
        cpu.reg[3] = 0;             /* RdHi */
        /* op2_misc = 1011 (N=1, M=1): hw2 = RdLo<<12 | RdHi<<8 | 0xB0 | Rm */
        write_thumb32(&cpu, CODE_BASE, 0xFBC0, 0x23B1);
        arm_step(&cpu, 1);
        /* Product = 0x7000 * 0x4000 = 0x1C000000 (positive)
           Acc = 0xFFFFFFFF + 0x1C000000 = 0x1_1BFFFFFF */
        assert_eq("SMLALTT lo wrap", 0x1BFFFFFF, cpu.reg[2]);
        assert_eq("SMLALTT hi carries", 1, cpu.reg[3]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * Cortex-M4F VFP (FPv4-SP-D16) tests
 *
 * Each test pre-seeds vfp_s[] (and/or core regs) with a known value,
 * writes the VFP encoding into flash, runs one instruction, and
 * asserts the destination register / FPSCR.
 *
 * Encoding reference: ARMv7-M ARM A6.6, A7.7.224–A7.7.276.
 *
 * Common encoding shape for SP data-processing:
 *   hw1 = 1110_1110_opc1_Vn   (Vn = N:Vn<3:0>, N at hw1[7])
 *   hw2 = Vd_3:0_:1010_:N_opc3_M_0_Vm   (Vd = D:Vd<3:0>, D at hw1[6];
 *                                        Vm = M:Vm<3:0>, M at hw2[5])
 *
 * Helper: type-pun float ↔ uint32 so test sources can write raw bit
 * patterns without depending on endianness/conversion details.
 * =================================================================== */
#include <math.h>
#include <string.h>

static uint32_t f_to_u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static float    u_to_f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static void test_m4_vfp(void) {
    printf("--- M4 VFP (FPv4-SP-D16) tests ---\n");

    /* ---- VMOV S0, R0  (core → single) ----
     * hw1 = 1110_1110_0000_0000 = 0xEE00, hw2 = Rt(0)<<12 | 0x0A10 = 0x0A10 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x12345678;
        write_thumb32(&cpu, CODE_BASE, 0xEE00, 0x0A10);
        arm_step(&cpu, 1);
        assert_eq("VMOV S0, R0 (raw bits preserved)", 0x12345678, cpu.vfp_s[0]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VMOV R1, S2  (single → core) ----
     * Sn=2 → Vn<3:0>=1 (Vn>>1), N=0. op=1.
     * hw1 = 1110_1110_0001_0001 = 0xEE11
     * hw2 = Rt(1)<<12 | 0x0A10 = 0x1A10 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[2] = 0xCAFEBABE;
        write_thumb32(&cpu, CODE_BASE, 0xEE11, 0x1A10);
        arm_step(&cpu, 1);
        assert_eq("VMOV R1, S2 (raw bits preserved)", 0xCAFEBABE, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VPUSH {S0, S1}  (push 2 SP regs) ----
     * hw1 = 1110_1101_0010_1101 = 0xED2D (D=0, Vd<5>=0)
     * hw2 = Vd<3:0>(0)<<12 | 0x0A02 = 0x0A02   (imm8 = 2 single regs) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[0] = 0x11111111;
        cpu.vfp_s[1] = 0x22222222;
        uint32_t sp_before = cpu.reg[13];
        write_thumb32(&cpu, CODE_BASE, 0xED2D, 0x0A02);
        arm_step(&cpu, 1);
        assert_eq("VPUSH {S0,S1} adjusts SP -8",
                  sp_before - 8, cpu.reg[13]);
        assert_eq("VPUSH {S0,S1} stored S0 at [SP]",
                  0x11111111, arm_read32(&cpu, cpu.reg[13]));
        assert_eq("VPUSH {S0,S1} stored S1 at [SP+4]",
                  0x22222222, arm_read32(&cpu, cpu.reg[13] + 4));
        arm_cpu_destroy(&cpu);
    }

    /* ---- VPOP {S0, S1}  (pop 2 SP regs) ----
     * hw1 = 1110_1100_1011_1101 = 0xECBD, hw2 = 0x0A02 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        /* setup_arm leaves SP at SRAM end (0x20008000) — pointing one
         * past the last byte. Move SP into SRAM and seed the popped data. */
        uint32_t sp = ARM_SRAM_BASE + 0x100;
        cpu.reg[13] = sp;
        arm_write32(&cpu, sp,     0x55556666);
        arm_write32(&cpu, sp + 4, 0x77778888);
        write_thumb32(&cpu, CODE_BASE, 0xECBD, 0x0A02);
        arm_step(&cpu, 1);
        assert_eq("VPOP {S0,S1} adjusts SP +8", sp + 8, cpu.reg[13]);
        assert_eq("VPOP {S0,S1} loaded S0 from [SP]", 0x55556666, cpu.vfp_s[0]);
        assert_eq("VPOP {S0,S1} loaded S1 from [SP+4]", 0x77778888, cpu.vfp_s[1]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VPUSH {D8} = VPUSH {S16,S17}  (coproc=B alias) ----
     * hw1 = 0xED2D (D=0, Vd<5>=0)
     * hw2 = Vd<3:0>(8)<<12 | 0x0B02 = 0x8B02  (coproc=B, imm8 = 1 D-reg = 2 S-regs) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[16] = 0xAAAAAAAA;
        cpu.vfp_s[17] = 0xBBBBBBBB;
        uint32_t sp_before = cpu.reg[13];
        write_thumb32(&cpu, CODE_BASE, 0xED2D, 0x8B02);
        arm_step(&cpu, 1);
        assert_eq("VPUSH {D8}: SP -= 8", sp_before - 8, cpu.reg[13]);
        assert_eq("VPUSH {D8}: S16 stored",
                  0xAAAAAAAA, arm_read32(&cpu, cpu.reg[13]));
        assert_eq("VPUSH {D8}: S17 stored",
                  0xBBBBBBBB, arm_read32(&cpu, cpu.reg[13] + 4));
        arm_cpu_destroy(&cpu);
    }

    /* ---- VLDR S0, [R0, #0]  (single load from immediate-addressed mem) ----
     * hw1 = 1110_1101_1001_0000 = 0xED90  (U=1, D=0, L=1, Rn=0)
     * hw2 = Vd<3:0>(0)<<12 | 0x0A00 (imm8 = 0) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t scratch = ARM_SRAM_BASE + 0x100;
        arm_write32(&cpu, scratch, 0xDEADC0DE);
        cpu.reg[0] = scratch;
        write_thumb32(&cpu, CODE_BASE, 0xED90, 0x0A00);
        arm_step(&cpu, 1);
        assert_eq("VLDR S0, [R0]", 0xDEADC0DE, cpu.vfp_s[0]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VSTR S1, [R0, #4] ----
     * Sd=1 → Vd<3:0>=0, D=1. hw1 = 0xED80 | (D<<6)=0x40 | Rn(0) = 0xEDC0.
     * hw2 = Vd<3:0>(0)<<12 | 0x0A01 (imm8 = 1 → offset 4) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t scratch = ARM_SRAM_BASE + 0x100;
        cpu.reg[0] = scratch;
        cpu.vfp_s[1] = 0xFEEDFACE;
        write_thumb32(&cpu, CODE_BASE, 0xEDC0, 0x0A01);
        arm_step(&cpu, 1);
        assert_eq("VSTR S1, [R0,#4]", 0xFEEDFACE, arm_read32(&cpu, scratch + 4));
        arm_cpu_destroy(&cpu);
    }

    /* ---- VADD.F32 S0, S1, S2 ----
     * Sd=0 (D=0,Vd[3:0]=0); Sn=1 (N=1,Vn[3:0]=0); Sm=2 (M=0,Vm[3:0]=1).
     * hw1 = 1110_1110_0011_0000 | (N<<7) = 0xEE30 | 0x80 = 0xEE30 with N at bit 7.
     * Actually N is at hw1[7]. Sn=1 means Vn<5>=N=1, Vn<3:0>=0. So hw1 bit 7 = 1.
     * hw1 = 1110_1110_0011_0000 with bit 7 set = 0xEE30 | 0x80? wait that overlaps with opc1.
     * Let me recompute: hw1 = 0xEE30 base for VADD opc1==0x3. Vn<3:0> at hw1[3:0] = 0.
     * N at hw1[7] = 1. So hw1 = 0xEE30 with bit 7 set:
     *   0xEE30 = 1110_1110_0011_0000 — bit 7 is 0 here.
     *   With N=1: 0xEE30 | 0x0080 = 0xEEB0... wait that overlaps with opc1=0xB.
     * The encoding is hw1[7] for Vn[5] (the N bit). Bit 7 in 0xEE30 = 0. Setting it gives 0xEEB0
     * which conflicts with the misc-DP opc1=0xB encoding!
     *
     * To stay outside the misc-DP space, use Sn=0 (N=0). Sd=2, Sn=0, Sm=4:
     *   Sd=2: D=0, Vd[3:0]=1
     *   Sn=0: N=0, Vn[3:0]=0
     *   Sm=4: M=0, Vm[3:0]=2
     * hw1 = 0xEE30 (opc1=3, N=0, Vn[3:0]=0)
     * hw2 = Vd[3:0](1)<<12 | 0x0A00 | (M<<5) | Vm[3:0](2) = 0x1A02
     */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[0] = f_to_u(2.5f);
        cpu.vfp_s[4] = f_to_u(1.5f);
        write_thumb32(&cpu, CODE_BASE, 0xEE30, 0x1A02);
        arm_step(&cpu, 1);
        assert_eq("VADD.F32 S2, S0, S4 = 2.5 + 1.5",
                  f_to_u(4.0f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VSUB.F32 S2, S0, S4 ---- (opc3 bit 6 = 1 → subtract)
     * hw2 = Vd<<12 | 0x0A00 | (1<<6) | (M<<5) | Vm = 0x1A42 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[0] = f_to_u(5.0f);
        cpu.vfp_s[4] = f_to_u(1.25f);
        write_thumb32(&cpu, CODE_BASE, 0xEE30, 0x1A42);
        arm_step(&cpu, 1);
        assert_eq("VSUB.F32 S2 = 5.0 - 1.25", f_to_u(3.75f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VMUL.F32 S2, S0, S4 ----
     * opc1 = 0x2 → hw1 = 0xEE20, hw2 = 0x1A02 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[0] = f_to_u(3.0f);
        cpu.vfp_s[4] = f_to_u(2.5f);
        write_thumb32(&cpu, CODE_BASE, 0xEE20, 0x1A02);
        arm_step(&cpu, 1);
        assert_eq("VMUL.F32 S2 = 3.0 * 2.5", f_to_u(7.5f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VDIV.F32 S2, S0, S4 ----
     * opc1 = 0x8 → hw1 = 0xEE80, hw2 = 0x1A02 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[0] = f_to_u(9.0f);
        cpu.vfp_s[4] = f_to_u(4.0f);
        write_thumb32(&cpu, CODE_BASE, 0xEE80, 0x1A02);
        arm_step(&cpu, 1);
        assert_eq("VDIV.F32 S2 = 9.0 / 4.0", f_to_u(2.25f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VABS.F32 S2, S4 ----  (opc1=B, opc2=0, opc3=3 → VABS)
     * hw1 = 0xEEB0 (opc1=B, Vn ignored), hw2 = Vd(1)<<12 | 0x0AC0 | Vm(2) = 0x1AC2 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[4] = f_to_u(-3.5f);
        write_thumb32(&cpu, CODE_BASE, 0xEEB0, 0x1AC2);
        arm_step(&cpu, 1);
        assert_eq("VABS.F32 S2 = abs(-3.5)", f_to_u(3.5f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VNEG.F32 S2, S4 ----  (opc1=B, opc2=1, opc3=1 → VNEG)
     * hw1 = 0xEEB1, hw2 = Vd(1)<<12 | 0x0A40 | Vm(2) = 0x1A42 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[4] = f_to_u(7.25f);
        write_thumb32(&cpu, CODE_BASE, 0xEEB1, 0x1A42);
        arm_step(&cpu, 1);
        assert_eq("VNEG.F32 S2 = -7.25", f_to_u(-7.25f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VSQRT.F32 S2, S4 ----  (opc1=B, opc2=1, opc3=3 → VSQRT)
     * hw1 = 0xEEB1, hw2 = Vd(1)<<12 | 0x0AC0 | Vm(2) = 0x1AC2 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[4] = f_to_u(16.0f);
        write_thumb32(&cpu, CODE_BASE, 0xEEB1, 0x1AC2);
        arm_step(&cpu, 1);
        assert_eq("VSQRT.F32 S2 = sqrt(16) = 4.0", f_to_u(4.0f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VCMP.F32 S2, S4 ----  (opc1=B, opc2=4, opc3=1)
     * Compare 2.0 vs 1.0 → greater → FPSCR NZCV = 0010 (C=1, Z=0, N=0, V=0) → 0x20000000.
     * hw1 = 0xEEB4, hw2 = Vd(1)<<12 | 0x0A40 | Vm(2) = 0x1A42 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[2] = f_to_u(2.0f);
        cpu.vfp_s[4] = f_to_u(1.0f);
        write_thumb32(&cpu, CODE_BASE, 0xEEB4, 0x1A42);
        arm_step(&cpu, 1);
        assert_eq("VCMP 2.0 > 1.0 → FPSCR.NZCV = 0010 (C set)",
                  0x20000000u, cpu.fpscr & 0xF0000000u);
        arm_cpu_destroy(&cpu);
    }

    /* VCMP equal */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[2] = f_to_u(1.5f);
        cpu.vfp_s[4] = f_to_u(1.5f);
        write_thumb32(&cpu, CODE_BASE, 0xEEB4, 0x1A42);
        arm_step(&cpu, 1);
        assert_eq("VCMP 1.5 == 1.5 → FPSCR.NZCV = 0110 (Z+C)",
                  0x60000000u, cpu.fpscr & 0xF0000000u);
        arm_cpu_destroy(&cpu);
    }

    /* VCMP less */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[2] = f_to_u(-1.0f);
        cpu.vfp_s[4] = f_to_u(0.5f);
        write_thumb32(&cpu, CODE_BASE, 0xEEB4, 0x1A42);
        arm_step(&cpu, 1);
        assert_eq("VCMP -1.0 < 0.5 → FPSCR.NZCV = 1000 (N set)",
                  0x80000000u, cpu.fpscr & 0xF0000000u);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VCVT.F32.S32 S2, S4 ----  (signed int → float; opc1=B, opc2=8)
     * hw1 = 0xEEB8, hw2 = Vd(1)<<12 | 0x0AC0 | Vm(2) = 0x1AC2 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[4] = (uint32_t)-42;
        write_thumb32(&cpu, CODE_BASE, 0xEEB8, 0x1AC2);
        arm_step(&cpu, 1);
        assert_eq("VCVT.F32.S32 S2 = float(-42)", f_to_u(-42.0f), cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VCVT.S32.F32 S2, S4 ----  (float → signed int; opc1=B, opc2=D, RTZ)
     * hw1 = 0xEEBD, hw2 = Vd(1)<<12 | 0x0AC0 | Vm(2) = 0x1AC2 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[4] = f_to_u(-7.9f);
        write_thumb32(&cpu, CODE_BASE, 0xEEBD, 0x1AC2);
        arm_step(&cpu, 1);
        /* Round-to-zero of -7.9 is -7. */
        assert_eq("VCVT.S32.F32 S2 = int(-7.9) RTZ = -7",
                  (uint32_t)(int32_t)-7, cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- VMOV.F32 S2, S4 ----  (opc1=B, opc2=0, opc3=1 → VMOV)
     * hw1 = 0xEEB0, hw2 = Vd(1)<<12 | 0x0A40 | Vm(2) = 0x1A42 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.vfp_s[4] = 0xCAFEF00D;
        write_thumb32(&cpu, CODE_BASE, 0xEEB0, 0x1A42);
        arm_step(&cpu, 1);
        assert_eq("VMOV.F32 S2, S4 (raw bits)", 0xCAFEF00D, cpu.vfp_s[2]);
        arm_cpu_destroy(&cpu);
    }

    /* ---- Unused helper to silence -Wunused-function ---- */
    (void)u_to_f;
}

/* ===================================================================
 * Run all tests
 * =================================================================== */
int run_arm_correctness_tests(int v) {
    printf("=== ARM Cortex-M3/M4 Correctness Tests ===\n\n");
    passed = 0;
    failed = 0;
    verbose = v;

    test_mov();
    test_add_sub();
    test_cmp();
    test_logic();
    test_shifts();
    test_load_store();
    test_branch();
    test_extensions();
    test_adc_sbc();
    test_m4_dsp_halfword_multiply();
    test_m4_vfp();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
