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
 * Run all tests
 * =================================================================== */
int run_arm_correctness_tests(int v) {
    printf("=== ARM Cortex-M3 Correctness Tests ===\n\n");
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

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
