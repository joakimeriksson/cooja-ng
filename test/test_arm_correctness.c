/*
 * Correctness tests for the ARM Cortex-M3 CPU emulator.
 */
#include "arm_cpu.h"
#include "arm_config.h"
#include "arm_trustzone.h"
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

/* Helper: set up CPU with code at flash base.  Writes the vector table
 * directly into the flash array; flash is read-only via arm_write32. */
static void write_flash32(arm_cpu_t *cpu, uint32_t addr, uint32_t val) {
    uint32_t off = addr - ARM_FLASH_BASE;
    cpu->flash[off]     = val & 0xFF;
    cpu->flash[off + 1] = (val >> 8) & 0xFF;
    cpu->flash[off + 2] = (val >> 16) & 0xFF;
    cpu->flash[off + 3] = (val >> 24) & 0xFF;
}

static void setup_arm(arm_cpu_t *cpu) {
    arm_cpu_init(cpu, &cc2538_config);
    /* Set initial SP */
    write_flash32(cpu, ARM_FLASH_BASE, ARM_SRAM_BASE + ARM_SRAM_SIZE);
    /* Set initial PC (entry point) - with thumb bit */
    write_flash32(cpu, ARM_FLASH_BASE + 4, ARM_FLASH_BASE + 0x100 + 1);
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

    /* ADC V-flag with carry-in crossing the sign boundary.
     * 0x80000000 + 0x7FFFFFFF + C(1) = 0 : signed (-2^31)+(2^31-1)+1 = 0, no
     * overflow → V must be 0. The bug folded carry-in into operand b
     * (0x7FFFFFFF+1=0x80000000), flipping its sign bit and yielding V=1. */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        write_thumb16(&cpu, CODE_BASE, 0x4148); /* ADCS r0, r1 */
        cpu.reg[0] = 0x80000000;
        cpu.reg[1] = 0x7FFFFFFF;
        cpu.xpsr |= APSR_C;                     /* carry-in = 1 */
        arm_step(&cpu, 1);
        assert_eq("ADCS r0,r1 (0x80000000+0x7FFFFFFF+1) = 0", 0, cpu.reg[0]);
        assert_true("ADCS sets Z", (cpu.xpsr & APSR_Z) != 0);
        assert_true("ADCS sets C", (cpu.xpsr & APSR_C) != 0);
        assert_true("ADCS V clear (no signed overflow)", (cpu.xpsr & APSR_V) == 0);
        arm_cpu_destroy(&cpu);
    }

    /* Same boundary case for the 32-bit ADC.W (register form). */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        write_thumb32(&cpu, CODE_BASE, 0xEB50, 0x0001); /* ADCS.W r0, r0, r1 */
        cpu.reg[0] = 0x80000000;
        cpu.reg[1] = 0x7FFFFFFF;
        cpu.xpsr |= APSR_C;
        arm_step(&cpu, 1);
        assert_eq("ADCS.W r0,r0,r1 = 0", 0, cpu.reg[0]);
        assert_true("ADCS.W V clear (no signed overflow)", (cpu.xpsr & APSR_V) == 0);
        arm_cpu_destroy(&cpu);
    }

    /* SDIV INT_MIN/-1 must yield INT_MIN (ARMv7-M), not crash (C UB → SIGFPE). */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        write_thumb32(&cpu, CODE_BASE, 0xFB91, 0xF0F2); /* SDIV r0, r1, r2 */
        cpu.reg[1] = 0x80000000;
        cpu.reg[2] = 0xFFFFFFFF;
        arm_step(&cpu, 1);
        assert_eq("SDIV INT_MIN/-1 = INT_MIN", 0x80000000, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * UMULL (unsigned 64-bit multiply) tests
 *
 * Encoding: UMULL RdLo, RdHi, Rn, Rm  (T1)
 *   hw1 = 0xFBA0 | Rn
 *   hw2 = (RdLo<<12) | (RdHi<<8) | 0x0000 | Rm
 *
 * Used heavily in uECC's FAST_MULT_ASM multi-precision field arithmetic.
 * =================================================================== */
static void test_umull(void) {
    printf("--- UMULL (unsigned 64-bit multiply) tests ---\n");

    /* Basic: 3 * 7 = 21, fits in low word */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2003); pc += 2; /* MOVS r0, #3 */
        write_thumb16(&cpu, pc, 0x2107); pc += 2; /* MOVS r1, #7 */
        /* UMULL r2, r3, r0, r1 — hw1=0xFBA0 (Rn=r0), hw2=0x2301 (RdLo=r2,RdHi=r3,Rm=r1) */
        write_thumb32(&cpu, pc, 0xFBA0, 0x2301); pc += 4;
        arm_step(&cpu, 3);
        assert_eq("UMULL r2,r3,r0,r1: lo=21", 21, cpu.reg[2]);
        assert_eq("UMULL r2,r3,r0,r1: hi=0",   0, cpu.reg[3]);
        arm_cpu_destroy(&cpu);
    }

    /* Overflow into high word: 0xFFFFFFFF * 0xFFFFFFFF */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        /* MOVS r0, #1; MVN r0, r0 → r0 = 0xFFFFFFFE; not clean. Use MOV.W instead */
        /* MVN r0, #0 sets r0 = 0xFFFFFFFF */
        write_thumb32(&cpu, pc, 0xF06F, 0x0000); pc += 4; /* MVN r0, #0 → r0=0xFFFFFFFF */
        write_thumb32(&cpu, pc, 0xF06F, 0x0100); pc += 4; /* MVN r1, #0 → r1=0xFFFFFFFF */
        /* UMULL r2, r3, r0, r1 */
        write_thumb32(&cpu, pc, 0xFBA0, 0x2301); pc += 4;
        arm_step(&cpu, 3);
        /* 0xFFFFFFFF * 0xFFFFFFFF = 0xFFFFFFFE_00000001 */
        assert_eq("UMULL 0xFFFFFF*0xFFFFFF: lo=1",          0x00000001, cpu.reg[2]);
        assert_eq("UMULL 0xFFFFFF*0xFFFFFF: hi=0xFFFFFFFE", 0xFFFFFFFE, cpu.reg[3]);
        arm_cpu_destroy(&cpu);
    }

    /* Source and destination register aliasing: UMULL r0, r1, r0, r1
     * Reads r0 and r1 first, then writes — must not corrupt inputs before compute */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2064); pc += 2; /* MOVS r0, #100 */
        write_thumb16(&cpu, pc, 0x2132); pc += 2; /* MOVS r1, #50  */
        /* UMULL r0, r1, r0, r1 — RdLo=r0,RdHi=r1,Rn=r0,Rm=r1 */
        write_thumb32(&cpu, pc, 0xFBA0, 0x0101); pc += 4;
        arm_step(&cpu, 3);
        /* 100 * 50 = 5000 = 0x1388 */
        assert_eq("UMULL r0,r1,r0,r1 (aliased): lo=5000", 5000, cpu.reg[0]);
        assert_eq("UMULL r0,r1,r0,r1 (aliased): hi=0",      0,   cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* UMULL with high-register destination: r8 (RdLo), r9 (RdHi) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2006); pc += 2; /* MOVS r0, #6  */
        write_thumb16(&cpu, pc, 0x2107); pc += 2; /* MOVS r1, #7  */
        /* UMULL r8, r9, r0, r1 — hw1=0xFBA0 (Rn=r0), hw2=(8<<12)|(9<<8)|1=0x8901 */
        write_thumb32(&cpu, pc, 0xFBA0, 0x8901); pc += 4;
        arm_step(&cpu, 3);
        assert_eq("UMULL r8,r9,r0,r1: lo=42", 42, cpu.reg[8]);
        assert_eq("UMULL r8,r9,r0,r1: hi=0",   0, cpu.reg[9]);
        arm_cpu_destroy(&cpu);
    }

    /* ADC carry chain after UMULL+ADDS+ADCS (matches FAST_MULT_ASM pattern) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        /* c0=0, c1=0, c2=0 → multiply t0,t1 = 0xFFFFFFFF * 0xFFFFFFFF, add to accumulators */
        write_thumb32(&cpu, pc, 0xF04F, 0x0300); pc += 4; /* MOV r3, #0   (c0) */
        write_thumb32(&cpu, pc, 0xF04F, 0x0400); pc += 4; /* MOV r4, #0   (c1) */
        write_thumb32(&cpu, pc, 0xF04F, 0x0500); pc += 4; /* MOV r5, #0   (c2) */
        write_thumb32(&cpu, pc, 0xF06F, 0x0600); pc += 4; /* MVN r6, #0   (t0 source = 0xFFFFFFFF) */
        write_thumb32(&cpu, pc, 0xF06F, 0x0700); pc += 4; /* MVN r7, #0   (t1 source = 0xFFFFFFFF) */
        /* UMULL r6, r7, r6, r7 → (t0,t1) = 0xFFFFFFFE_00000001 */
        write_thumb32(&cpu, pc, 0xFBA6, 0x6707); pc += 4;
        /* ADDS r3, r3, r6 → c0 += t0_lo; sets carry */
        write_thumb32(&cpu, pc, 0xEB13, 0x0306); pc += 4;
        /* ADCS r4, r4, r7 → c1 += t0_hi + carry */
        write_thumb32(&cpu, pc, 0xEB54, 0x0407); pc += 4;
        /* ADC r5, r5, #0 → c2 += carry (modified-imm T1: hw1=0xF145,hw2=0x0500) */
        write_thumb32(&cpu, pc, 0xF145, 0x0500); pc += 4;
        arm_step(&cpu, 9);
        /* c0 = 0x00000001, c1 = 0xFFFFFFFE, c2 = 0 */
        assert_eq("FAST_MULT pattern: c0=1",          0x00000001, cpu.reg[3]);
        assert_eq("FAST_MULT pattern: c1=0xFFFFFFFE", 0xFFFFFFFE, cpu.reg[4]);
        assert_eq("FAST_MULT pattern: c2=0",          0,          cpu.reg[5]);
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
                  (uint32_t)0x7FFFFFFF + (uint32_t)0x3FFF0001, cpu.reg[2]);
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
/* ===================================================================
 * Bit field instruction tests — BFI / BFC / SBFX / UBFX
 *
 * Regression cover for the nrf54l15 channel bitfield bug: BFI's op=0x16
 * was previously matched with `(op & 0x1C) == 0x18`, which captures
 * 0x18..0x1B but NOT 0x16, so every BFI silently fell through to the
 * SBFX path (0x14..0x17) and corrupted any bitfield write.  Caught
 * because Nordic's `nrf_802154` PIB stores the IEEE 802.15.4 channel
 * as `uint8_t channel : 5` and the channel-setter compiles to
 *   bfi r2, r0, #3, #5
 * which silently became sbfx, leaving the channel at 0 in SRAM and
 * tripping `NRF_802154_ASSERT(channel >= 11 && channel <= 26)` at
 * trx_enable time.
 * =================================================================== */
static void test_bit_field_ops(void) {
    printf("--- Bit-field instruction tests ---\n");

    /* BFI R2, R0, #3, #5
     * Insert 5 bits from R0 at bit position 3 of R2.
     * Encoding: hw1=0xF360 (op=0x16, Rn=R0)
     *           hw2 = (imm3=0)<<12 | (Rd=R2)<<8 | (imm2=3)<<6 | (msb=7) = 0x02C7
     */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x201A); pc += 2;   /* MOVS R0, #26 */
        write_thumb16(&cpu, pc, 0x2207); pc += 2;   /* MOVS R2, #7  (preserve low 3 bits) */
        write_thumb32(&cpu, pc, 0xF360, 0x02C7); pc += 4; /* BFI R2, R0, #3, #5 */
        arm_step(&cpu, 3);
        /* expected: R2 = (7 & 0x07) | ((26 << 3) & 0xF8) = 7 | 0xD0 = 0xD7 */
        assert_eq("BFI R2, R0, #3, #5  (26 into 5-bit field)", 0xD7, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* BFC R2, #4, #4 — clear 4 bits of R2 starting at bit 4
     * lsb=4 → imm3=1, imm2=0.  msb=lsb+width-1=7.
     * Encoding: hw1 = 0xF36F (op=0x16, Rn=PC=0xF marks BFC)
     *           hw2 = (imm3=1)<<12 | (Rd=R2)<<8 | (imm2=0)<<6 | (msb=7) = 0x1207
     */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x22FF); pc += 2;   /* MOVS R2, #0xFF */
        write_thumb32(&cpu, pc, 0xF36F, 0x1207); pc += 4; /* BFC R2, #4, #4 */
        arm_step(&cpu, 2);
        assert_eq("BFC R2, #4, #4 (clear bits 4..7 of 0xFF)", 0x0F, cpu.reg[2]);
        arm_cpu_destroy(&cpu);
    }

    /* UBFX R1, R0, #3, #5 — unsigned extract 5 bits at bit 3
     * Encoding: hw1 = 0xF3C0 (op=0x1C, Rn=R0)
     *           hw2 = (imm3=0)<<12 | (Rd=R1)<<8 | (imm2=3)<<6 | (widthm1=4) = 0x01C4
     */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x20D0); pc += 2;   /* MOVS R0, #0xD0 */
        write_thumb32(&cpu, pc, 0xF3C0, 0x01C4); pc += 4; /* UBFX R1, R0, #3, #5 */
        arm_step(&cpu, 2);
        /* expected: (0xD0 >> 3) & 0x1F = 0x1A = 26 */
        assert_eq("UBFX R1, R0, #3, #5  (extract 5 bits at offset 3)", 26, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* SBFX R1, R0, #3, #5 — signed extract 5 bits at bit 3
     * Encoding: hw1 = 0xF340 (op=0x14, Rn=R0)
     *           hw2 = same shape as UBFX = 0x01C4
     */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t pc = CODE_BASE;
        write_thumb16(&cpu, pc, 0x2080); pc += 2;   /* MOVS R0, #0x80 (bit 7 set) */
        write_thumb32(&cpu, pc, 0xF340, 0x01C4); pc += 4; /* SBFX R1, R0, #3, #5 */
        arm_step(&cpu, 2);
        /* extract bits 3..7 of 0x80 = 0b10000 = 16, sign-extend (top bit is 1)
         * → 0xFFFFFFF0 */
        assert_eq("SBFX R1, R0, #3, #5  (sign-extend extracted negative)",
                  0xFFFFFFF0u, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * ARMv8-M atomic acquire/release exclusive — LDAEX / STLEX
 *
 * Regression cover for the nrf54l15 GRTC tick handler bug: Nordic's
 * `nrfx_atomic_u32_fetch_and` uses
 *   LDAEX r0, [r3] ; ... ; STLEX ip, r2, [r3]
 * to read-modify-write a 32-bit flag.  Without LDAEX/STLEX decoding,
 * the interpreter mis-decoded the instructions and ended up jumping
 * to PC=0, which silently wiped SRAM during the IRQ handler path.
 * =================================================================== */
static void test_ldaex_stlex(void) {
    printf("--- ARMv8-M LDAEX / STLEX tests ---\n");

    /* Set up R3 = 0x20001000 directly via cpu->reg, then run the
     * exclusive instructions.  Saves the headache of encoding
     * MOVW/MOVT by hand. */

    /* LDAEX R0, [R3]
     * Encoding: hw1=0xE8D3 (LDAEX with Rn=R3), hw2=0x0FEF (Rt=R0, marker). */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        arm_write32(&cpu, 0x20001000, 0xDEADBEEFu);
        cpu.reg[3] = 0x20001000;
        write_thumb32(&cpu, CODE_BASE, 0xE8D3, 0x0FEF);
        arm_step(&cpu, 1);
        assert_eq("LDAEX R0, [R3] (R0 = *0x20001000)", 0xDEADBEEFu, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* STLEX IP, R2, [R3]
     * Encoding: hw1=0xE8C3 (STLEX with Rn=R3), hw2=0x2FEC (Rt=R2, marker, Rd=R12=IP). */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[3] = 0x20001000;
        cpu.reg[2] = 0x12345678;
        cpu.reg[12] = 0xFFFFFFFF;   /* will be cleared to 0 on success */
        write_thumb32(&cpu, CODE_BASE, 0xE8C3, 0x2FEC);
        arm_step(&cpu, 1);
        assert_eq("STLEX IP = 0 (always succeeds in csim)", 0u, cpu.reg[12]);
        assert_eq("STLEX wrote *0x20001000 = R2", 0x12345678u,
                  arm_read32(&cpu, 0x20001000));
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * LDRD/STRD and 64-bit arithmetic tests
 *
 * These exercise the instruction patterns used by the Mbed TLS DTLS
 * anti-replay window (mbedtls_ssl_dtls_replay_check/update):
 *
 *   - LDRD/STRD at byte offsets 136 (in_window_top) and 144 (in_window)
 *     in the mbedtls_ssl_context struct.
 *   - CMP.W + SBCS.W for 64-bit unsigned comparison (rec_seqnum > in_window_top).
 *   - SUBS.W + SBC.W for 64-bit subtraction (bit = in_window_top - rec_seqnum).
 *   - LSL (register) for variable 64-bit window shift (in_window <<= shift).
 *
 * The ARM emulator had an LDRD/STRD double-shift bug: the decode block
 * pre-multiplied the field by 4 to get a byte offset, then the LDRD/STRD
 * sub-case multiplied again, giving offset*4 instead of offset.  The fix
 * sets `offset = imm8` (not `imm8 << 2`) in both LDRD-literal and
 * LDRD/STRD-immediate sub-cases of arm_cpu.c.
 * =================================================================== */

/* APSR flag bits in xpsr */
#define APSR_N_BIT (1u << 31)
#define APSR_Z_BIT (1u << 30)
#define APSR_C_BIT (1u << 29)
#define APSR_V_BIT (1u << 28)

static void test_anti_replay_ops(void) {
    printf("--- LDRD/STRD and 64-bit arithmetic (anti-replay) tests ---\n");

    /* ----------------------------------------------------------------
     * LDRD at offset 0 — sanity check that the base case works
     * LDRD R0, R1, [R2, #0]: hw1=0xE9D2, hw2=0x0100 (imm8_field=0)
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        arm_write32(&cpu, ARM_SRAM_BASE + 0, 0xAABBCCDD);
        arm_write32(&cpu, ARM_SRAM_BASE + 4, 0x11223344);
        cpu.reg[2] = ARM_SRAM_BASE;
        write_thumb32(&cpu, CODE_BASE, 0xE9D2, 0x0100);
        arm_step(&cpu, 1);
        assert_eq("LDRD offset=0: R0=mem[base+0]", 0xAABBCCDD, cpu.reg[0]);
        assert_eq("LDRD offset=0: R1=mem[base+4]", 0x11223344, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * LDRD at offset 136 (in_window_top in mbedtls_ssl_context)
     * LDRD R0, R1, [R2, #136]: imm8_field=34=0x22, hw2=0x0122
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        arm_write32(&cpu, ARM_SRAM_BASE + 136, 0xDEADBEEF);
        arm_write32(&cpu, ARM_SRAM_BASE + 140, 0x0000000F);
        cpu.reg[2] = ARM_SRAM_BASE;
        write_thumb32(&cpu, CODE_BASE, 0xE9D2, 0x0122);
        arm_step(&cpu, 1);
        assert_eq("LDRD offset=136: R0=mem[base+136]", 0xDEADBEEF, cpu.reg[0]);
        assert_eq("LDRD offset=136: R1=mem[base+140]", 0x0000000F, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * LDRD at offset 144 (in_window in mbedtls_ssl_context)
     * LDRD R0, R1, [R2, #144]: imm8_field=36=0x24, hw2=0x0124
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        arm_write32(&cpu, ARM_SRAM_BASE + 144, 0xCAFEBABE);
        arm_write32(&cpu, ARM_SRAM_BASE + 148, 0x12345678);
        cpu.reg[2] = ARM_SRAM_BASE;
        write_thumb32(&cpu, CODE_BASE, 0xE9D2, 0x0124);
        arm_step(&cpu, 1);
        assert_eq("LDRD offset=144: R0=mem[base+144]", 0xCAFEBABE, cpu.reg[0]);
        assert_eq("LDRD offset=144: R1=mem[base+148]", 0x12345678, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * STRD at offset 0 — sanity check
     * STRD R0, R1, [R2, #0]: hw1=0xE9C2, hw2=0x0100
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0xFEEDFACE;
        cpu.reg[1] = 0xBAADF00D;
        cpu.reg[2] = ARM_SRAM_BASE;
        write_thumb32(&cpu, CODE_BASE, 0xE9C2, 0x0100);
        arm_step(&cpu, 1);
        assert_eq("STRD offset=0: mem[base+0]=R0", 0xFEEDFACE, arm_read32(&cpu, ARM_SRAM_BASE + 0));
        assert_eq("STRD offset=0: mem[base+4]=R1", 0xBAADF00D, arm_read32(&cpu, ARM_SRAM_BASE + 4));
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * STRD at offset 136 (in_window_top)
     * STRD R0, R1, [R2, #136]: hw1=0xE9C2, hw2=0x0122
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00000005;  /* in_window_top low word = seqnum 5 */
        cpu.reg[1] = 0x00000000;  /* in_window_top high word = 0 */
        cpu.reg[2] = ARM_SRAM_BASE;
        write_thumb32(&cpu, CODE_BASE, 0xE9C2, 0x0122);
        arm_step(&cpu, 1);
        assert_eq("STRD offset=136: mem[base+136]=R0", 0x00000005, arm_read32(&cpu, ARM_SRAM_BASE + 136));
        assert_eq("STRD offset=136: mem[base+140]=R1", 0x00000000, arm_read32(&cpu, ARM_SRAM_BASE + 140));
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * STRD at offset 144 (in_window)
     * STRD R0, R1, [R2, #144]: hw1=0xE9C2, hw2=0x0124
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0x00000001;  /* in_window low word = bit 0 set */
        cpu.reg[1] = 0x00000000;  /* in_window high word = 0 */
        cpu.reg[2] = ARM_SRAM_BASE;
        write_thumb32(&cpu, CODE_BASE, 0xE9C2, 0x0124);
        arm_step(&cpu, 1);
        assert_eq("STRD offset=144: mem[base+144]=R0", 0x00000001, arm_read32(&cpu, ARM_SRAM_BASE + 144));
        assert_eq("STRD offset=144: mem[base+148]=R1", 0x00000000, arm_read32(&cpu, ARM_SRAM_BASE + 148));
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * LDRD round-trip: write via STRD, read back via LDRD (offset 144)
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0xABCD1234;
        cpu.reg[1] = 0x9876FEDC;
        cpu.reg[2] = ARM_SRAM_BASE;
        /* STRD R0, R1, [R2, #144] */
        write_thumb32(&cpu, CODE_BASE + 0, 0xE9C2, 0x0124);
        /* Clear R0 and R1 */
        write_thumb16(&cpu, CODE_BASE + 4, 0x2000);  /* MOVS R0, #0 */
        write_thumb16(&cpu, CODE_BASE + 6, 0x2100);  /* MOVS R1, #0 */
        /* LDRD R0, R1, [R2, #144] */
        write_thumb32(&cpu, CODE_BASE + 8, 0xE9D2, 0x0124);
        arm_step(&cpu, 4);
        assert_eq("LDRD/STRD round-trip offset=144: R0", 0xABCD1234, cpu.reg[0]);
        assert_eq("LDRD/STRD round-trip offset=144: R1", 0x9876FEDC, cpu.reg[1]);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * CMP.W flags: A > B (unsigned 32-bit comparison)
     * CMP.W R0, R1: hw1=0xEBB0, hw2=0x0F01
     * Tests the building block for 64-bit high-word comparison.
     * A=5 > B=3: expect C=1, Z=0, N=0
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 5;
        cpu.reg[1] = 3;
        write_thumb32(&cpu, CODE_BASE, 0xEBB0, 0x0F01);
        arm_step(&cpu, 1);
        assert_true("CMP.W 5>3: C=1", (cpu.xpsr & APSR_C_BIT) != 0);
        assert_true("CMP.W 5>3: Z=0", (cpu.xpsr & APSR_Z_BIT) == 0);
        assert_true("CMP.W 5>3: N=0", (cpu.xpsr & APSR_N_BIT) == 0);
        arm_cpu_destroy(&cpu);
    }

    /* CMP.W: A < B: expect C=0 (borrow), N=1 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 3;
        cpu.reg[1] = 5;
        write_thumb32(&cpu, CODE_BASE, 0xEBB0, 0x0F01);
        arm_step(&cpu, 1);
        assert_true("CMP.W 3<5: C=0", (cpu.xpsr & APSR_C_BIT) == 0);
        assert_true("CMP.W 3<5: N=1", (cpu.xpsr & APSR_N_BIT) != 0);
        arm_cpu_destroy(&cpu);
    }

    /* CMP.W: A == B: expect C=1, Z=1 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 7;
        cpu.reg[1] = 7;
        write_thumb32(&cpu, CODE_BASE, 0xEBB0, 0x0F01);
        arm_step(&cpu, 1);
        assert_true("CMP.W 7==7: C=1", (cpu.xpsr & APSR_C_BIT) != 0);
        assert_true("CMP.W 7==7: Z=1", (cpu.xpsr & APSR_Z_BIT) != 0);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * SBCS.W: carry chain for 64-bit comparison
     * Pattern: CMP.W A_lo, B_lo; SBCS R5, A_hi, B_hi
     * Tests that C flag from CMP flows correctly into SBCS.
     *
     * Encoding:
     *   CMP.W R0, R2:  hw1=0xEBB0, hw2=0x0F02
     *   SBCS  R5, R3, R4: hw1=0xEB73, hw2=0x0504
     *     (op_dp=0xB=SBC, S=1, Rn=R3; Rd=5, Rm=4)
     *
     * Case 1: A={R3:R0}={0:5}, B={R4:R2}={0:3} → A > B (low differs)
     *   CMP.W 5, 3: C=1 (no borrow), Z=0
     *   SBCS R5, 0, 0: 0-0-0=0, C=1, Z=1
     *   → combined: C=1 (A >= B)
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 5;   /* A_lo */
        cpu.reg[3] = 0;   /* A_hi */
        cpu.reg[2] = 3;   /* B_lo */
        cpu.reg[4] = 0;   /* B_hi */
        /* CMP.W R0, R2 */
        write_thumb32(&cpu, CODE_BASE + 0, 0xEBB0, 0x0F02);
        /* SBCS R5, R3, R4 */
        write_thumb32(&cpu, CODE_BASE + 4, 0xEB73, 0x0504);
        arm_step(&cpu, 2);
        assert_eq("SBCS 64-bit A>B (lo differs): R5=0", 0u, cpu.reg[5]);
        assert_true("SBCS 64-bit A>B (lo differs): C=1", (cpu.xpsr & APSR_C_BIT) != 0);
        arm_cpu_destroy(&cpu);
    }

    /* Case 2: A={0:1}, B={0:2} → A < B
     *   CMP.W 1, 2: C=0 (borrow), N=1
     *   SBCS R5, 0, 0: 0-0-1(borrow)=0xFFFFFFFF, C=0
     *   → C=0 (A < B) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 1;
        cpu.reg[3] = 0;
        cpu.reg[2] = 2;
        cpu.reg[4] = 0;
        write_thumb32(&cpu, CODE_BASE + 0, 0xEBB0, 0x0F02);
        write_thumb32(&cpu, CODE_BASE + 4, 0xEB73, 0x0504);
        arm_step(&cpu, 2);
        assert_eq("SBCS 64-bit A<B (lo differs): R5=0xFFFFFFFF",
                  0xFFFFFFFFu, cpu.reg[5]);
        assert_true("SBCS 64-bit A<B: C=0", (cpu.xpsr & APSR_C_BIT) == 0);
        arm_cpu_destroy(&cpu);
    }

    /* Case 3: A={1:0}, B={0:0} → A > B (high word differs)
     *   CMP.W A_lo=0, B_lo=0: C=1, Z=1
     *   SBCS R5, A_hi=1, B_hi=0: 1-0-0=1, C=1, Z=0
     *   → C=1, Z=0 (A > B) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0;   /* A_lo */
        cpu.reg[3] = 1;   /* A_hi */
        cpu.reg[2] = 0;   /* B_lo */
        cpu.reg[4] = 0;   /* B_hi */
        write_thumb32(&cpu, CODE_BASE + 0, 0xEBB0, 0x0F02);
        write_thumb32(&cpu, CODE_BASE + 4, 0xEB73, 0x0504);
        arm_step(&cpu, 2);
        assert_eq("SBCS 64-bit A>B (hi differs): R5=1", 1u, cpu.reg[5]);
        assert_true("SBCS 64-bit A>B (hi differs): C=1",
                    (cpu.xpsr & APSR_C_BIT) != 0);
        assert_true("SBCS 64-bit A>B (hi differs): Z=0",
                    (cpu.xpsr & APSR_Z_BIT) == 0);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * SUBS.W + SBC.W: 64-bit unsigned subtraction
     * Computes {R5:R4} = {R3:R0} - {R4:R2} (64-bit)
     *
     * Encoding:
     *   SUBS.W R4, R0, R2: hw1=0xEBB0, hw2=0x0402
     *   SBC.W  R5, R3, R3_copy... need separate regs.
     *
     * Use: A_lo in R0, A_hi in R1, B_lo in R2, B_hi in R3
     *      Result_lo in R4, Result_hi in R5
     *   SUBS.W R4, R0, R2: hw1=0xEBB0, hw2=0x0402 (Rd=4, Rn=0, Rm=2)
     *   SBC.W  R5, R1, R3: hw1=0xEB61, hw2=0x0503 (S=0, Rd=5, Rn=1, Rm=3)
     *
     * Case 1: A={0:5} - B={0:3} = {0:2}
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 5;   /* A_lo */
        cpu.reg[1] = 0;   /* A_hi */
        cpu.reg[2] = 3;   /* B_lo */
        cpu.reg[3] = 0;   /* B_hi */
        /* SUBS.W R4, R0, R2 */
        write_thumb32(&cpu, CODE_BASE + 0, 0xEBB0, 0x0402);
        /* SBC.W R5, R1, R3 (no S flag update needed) */
        write_thumb32(&cpu, CODE_BASE + 4, 0xEB61, 0x0503);
        arm_step(&cpu, 2);
        assert_eq("64-bit SUBS/SBC: 5-3 low", 2u, cpu.reg[4]);
        assert_eq("64-bit SUBS/SBC: 5-3 high", 0u, cpu.reg[5]);
        arm_cpu_destroy(&cpu);
    }

    /* Case 2: A={0:1} - B={0:0} = {0:1} (seqnum=1, in_window_top=0, bit=1) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 1;   /* A_lo = rec_seqnum_lo */
        cpu.reg[1] = 0;   /* A_hi */
        cpu.reg[2] = 0;   /* B_lo = in_window_top_lo */
        cpu.reg[3] = 0;   /* B_hi */
        write_thumb32(&cpu, CODE_BASE + 0, 0xEBB0, 0x0402);
        write_thumb32(&cpu, CODE_BASE + 4, 0xEB61, 0x0503);
        arm_step(&cpu, 2);
        assert_eq("64-bit SUBS/SBC: 1-0 low (bit index)", 1u, cpu.reg[4]);
        assert_eq("64-bit SUBS/SBC: 1-0 high", 0u, cpu.reg[5]);
        arm_cpu_destroy(&cpu);
    }

    /* Case 3: Borrow propagation — A={0:0} - B={0:1} = {0xFFFFFFFF:0xFFFFFFFF}
     * Simulates what happens when replay_check reads bit index wrongly. */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[0] = 0;   /* A_lo */
        cpu.reg[1] = 0;   /* A_hi */
        cpu.reg[2] = 1;   /* B_lo */
        cpu.reg[3] = 0;   /* B_hi */
        write_thumb32(&cpu, CODE_BASE + 0, 0xEBB0, 0x0402);
        write_thumb32(&cpu, CODE_BASE + 4, 0xEB61, 0x0503);
        arm_step(&cpu, 2);
        assert_eq("64-bit borrow: 0-1 low = 0xFFFFFFFF", 0xFFFFFFFFu, cpu.reg[4]);
        assert_eq("64-bit borrow: 0-1 high = 0xFFFFFFFF", 0xFFFFFFFFu, cpu.reg[5]);
        arm_cpu_destroy(&cpu);
    }

    /* ----------------------------------------------------------------
     * LSL (register) T1: variable shift for 64-bit window update
     * LSLS R0, R1 (T1 Thumb-16): 0x4088 (op=0x02, Rm=R1, Rdn=R0)
     * Tests that `in_window <<= shift` doesn't lose bits.
     * ---------------------------------------------------------------- */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        /* Set R0=1, R1=3 → LSLS R0, R1 → R0 = 1 << 3 = 8 */
        write_thumb16(&cpu, CODE_BASE + 0, 0x2001);  /* MOVS R0, #1 */
        write_thumb16(&cpu, CODE_BASE + 2, 0x2103);  /* MOVS R1, #3 */
        write_thumb16(&cpu, CODE_BASE + 4, 0x4088);  /* LSLS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("LSL(reg): 1 << 3 = 8", 8u, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* LSL by 31 (shift to MSB) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        write_thumb16(&cpu, CODE_BASE + 0, 0x2001);  /* MOVS R0, #1 */
        write_thumb16(&cpu, CODE_BASE + 2, 0x211F);  /* MOVS R1, #31 */
        write_thumb16(&cpu, CODE_BASE + 4, 0x4088);  /* LSLS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("LSL(reg): 1 << 31 = 0x80000000", 0x80000000u, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* LSL by 32 — all bits shifted out, result = 0 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        write_thumb16(&cpu, CODE_BASE + 0, 0x2001);  /* MOVS R0, #1 */
        write_thumb16(&cpu, CODE_BASE + 2, 0x2120);  /* MOVS R1, #32 */
        write_thumb16(&cpu, CODE_BASE + 4, 0x4088);  /* LSLS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("LSL(reg): 1 << 32 = 0", 0u, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }

    /* LSL by 0 — identity */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        write_thumb16(&cpu, CODE_BASE + 0, 0x20AB);  /* MOVS R0, #0xAB */
        write_thumb16(&cpu, CODE_BASE + 2, 0x2100);  /* MOVS R1, #0 */
        write_thumb16(&cpu, CODE_BASE + 4, 0x4088);  /* LSLS R0, R1 */
        arm_step(&cpu, 3);
        assert_eq("LSL(reg): 0xAB << 0 = 0xAB", 0xABu, cpu.reg[0]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * SUBS/SBCS Rd==Rn carry correctness (regression for rd-aliases-rn bug).
 *
 * The bug: when rd==rn, the register was updated BEFORE set_sub_flags
 * read it, so the C flag was derived from the NEW value instead of the
 * original — corrupting borrow propagation in multi-word subtraction.
 * =================================================================== */
static void test_subs_rd_eq_rn(void) {
    printf("--- SUBS/SBCS Rd==Rn carry tests ---\n");

    /* 16-bit SUBS r5, r5, r6 (T1: rd=rn=5, rm=6): 2 - 3, should set C=0 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[5] = 2;
        cpu.reg[6] = 3;
        /* SUBS r5, r5, r6: 0001 101 110 101 101 = 0x1BAD */
        write_thumb16(&cpu, CODE_BASE, 0x1BAD);
        arm_step(&cpu, 1);
        assert_eq("SUBS r5,r5,r6 (2-3) result", (uint32_t)-1, cpu.reg[5]);
        assert_true("SUBS r5,r5,r6 (2-3) C=0 (borrow)", (cpu.xpsr & APSR_C) == 0);
        arm_cpu_destroy(&cpu);
    }

    /* SUBS r5, r5, r6 (T1): 5 - 3, should set C=1 (no borrow) */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[5] = 5;
        cpu.reg[6] = 3;
        write_thumb16(&cpu, CODE_BASE, 0x1BAD);
        arm_step(&cpu, 1);
        assert_eq("SUBS r5,r5,r6 (5-3) result", 2, cpu.reg[5]);
        assert_true("SUBS r5,r5,r6 (5-3) C=1 (no borrow)", (cpu.xpsr & APSR_C) != 0);
        arm_cpu_destroy(&cpu);
    }

    /* 32-bit SUBS.W r3, r3, r2 (Rd==Rn=3, Rm=2): 1 - 2, C=0 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[3] = 1;
        cpu.reg[2] = 2;
        /* SUBS.W r3, r3, r2: hw1=0xEBB3 (S=1, Rn=r3), hw2=0x0302 (Rd=r3, Rm=r2) */
        write_thumb32(&cpu, CODE_BASE, 0xEBB3, 0x0302);
        arm_step(&cpu, 1);
        assert_eq("SUBS.W r3,r3,r2 (1-2) result", (uint32_t)-1, cpu.reg[3]);
        assert_true("SUBS.W r3,r3,r2 (1-2) C=0", (cpu.xpsr & APSR_C) == 0);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * VFP double-precision VLDR/VSTR tests (regression for 4-byte-only bug).
 * =================================================================== */
static void test_vfp_double(void) {
    printf("--- VFP double-precision VLDR/VSTR tests ---\n");

    /* VSTR d7, [r0, #0] should write 8 bytes */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        /* Set d7 = {s14, s15} = {0xDEADBEEF, 0x12345678} */
        cpu.vfp_s[14] = 0xDEADBEEF;
        cpu.vfp_s[15] = 0x12345678;
        uint32_t addr = cpu.sram_base + 64;
        cpu.reg[0] = addr;
        /* VSTR d7, [r0]: hw1=0xED80 (U=1,D=0,L=0,Rn=0), hw2=0x7B00 (Vd=7,coproc=B,imm8=0) */
        write_thumb32(&cpu, CODE_BASE, 0xED80, 0x7B00);
        arm_step(&cpu, 1);
        assert_eq("VSTR d7 low  word", 0xDEADBEEFu, arm_read32(&cpu, addr));
        assert_eq("VSTR d7 high word", 0x12345678u, arm_read32(&cpu, addr + 4));
        arm_cpu_destroy(&cpu);
    }

    /* VLDR d7, [r0, #0] should read 8 bytes */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        uint32_t addr = cpu.sram_base + 64;
        arm_write32(&cpu, addr,     0xAABBCCDD);
        arm_write32(&cpu, addr + 4, 0x11223344);
        cpu.reg[0] = addr;
        /* VLDR d7, [r0]: hw1=0xED90, hw2=0x7B00 (L=1) */
        write_thumb32(&cpu, CODE_BASE, 0xED90, 0x7B00);
        arm_step(&cpu, 1);
        assert_eq("VLDR d7 low  word → s14", 0xAABBCCDDu, cpu.vfp_s[14]);
        assert_eq("VLDR d7 high word → s15", 0x11223344u, cpu.vfp_s[15]);
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * LDRD / STRD (load/store double register) tests
 * Used heavily in SHA-256 transform for loading state and W[] values.
 * =================================================================== */
static void test_ldrd_strd(void) {
    printf("--- LDRD/STRD instruction tests ---\n");

    /* STRD / LDRD round-trip via SRAM */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        /* Store two values via STRD r0, r1, [r2, #0]
         * hw1 = 0xE9C2 (STRD, P=1, U=1, W=0, Rn=r2)
         * hw2 = 0x0100 (Rt=r0, Rt2=r1, imm8=0) */
        uint32_t sram_addr = cpu.sram_base + 64;
        cpu.reg[0] = 0xDEADBEEF;
        cpu.reg[1] = 0x12345678;
        cpu.reg[2] = sram_addr;
        /* STRD r0, r1, [r2, #0]: P=1,U=1,W=0 → hw1=0xE9C2, hw2=0x0100 */
        write_thumb32(&cpu, CODE_BASE, 0xE9C2, 0x0100);
        /* LDRD r3, r4, [r2, #0] */
        write_thumb32(&cpu, CODE_BASE + 4, 0xE9D2, 0x3400);
        arm_step(&cpu, 2);
        assert_eq("STRD/LDRD round-trip: word0", 0xDEADBEEFu, cpu.reg[3]);
        assert_eq("STRD/LDRD round-trip: word1", 0x12345678u, cpu.reg[4]);
        arm_cpu_destroy(&cpu);
    }

    /* LDRD with immediate offset: [sp, #124] as in SHA-256 transform.
     * SHA-256 allocates 372 bytes on stack first (sub sp, #372).
     * Here we subtract 512 bytes to make room for the offset access. */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        /* SUB SP, #512 (0x200) to allocate stack space */
        write_thumb16(&cpu, CODE_BASE + 0, 0xB082); /* SUB SP, #8 (placeholder — use direct reg write) */
        /* Easier: adjust SP directly, then write test values */
        cpu.reg[ARM_SP] -= 512;
        uint32_t sp = cpu.reg[ARM_SP];
        arm_write32(&cpu, sp + 124, 0xAABBCCDD);
        arm_write32(&cpu, sp + 128, 0x11223344);
        /* LDRD r6, r11, [sp, #124]: P=1,U=1,W=0,L=1,Rn=13=sp
         * hw1 = 0xE9DD, hw2 = (6<<12)|(11<<8)|(31) = 0x6B1F */
        write_thumb32(&cpu, CODE_BASE, 0xE9DD, 0x6B1F);
        arm_step(&cpu, 1);
        assert_eq("LDRD [sp,#124]: r6", 0xAABBCCDDu, cpu.reg[6]);
        assert_eq("LDRD [sp,#124]: r11", 0x11223344u, cpu.reg[11]);
        arm_cpu_destroy(&cpu);
    }

    /* STRD r3, r2, [sp, #28] — stores h and g in SHA-256 */
    {
        arm_cpu_t cpu;
        setup_arm(&cpu);
        cpu.reg[3] = 0x5BE0CD19u;  /* SHA-256 initial h */
        cpu.reg[2] = 0x1F83D9ABu;  /* SHA-256 initial g */
        cpu.reg[ARM_SP] -= 512;
        uint32_t sp = cpu.reg[ARM_SP];
        /* STRD r3, r2, [sp, #28]: P=1,U=1,W=0,L=0,Rn=13=sp
         * hw1 = 0xE9CD, hw2 = (3<<12)|(2<<8)|7 = 0x3207 */
        write_thumb32(&cpu, CODE_BASE, 0xE9CD, 0x3207);
        arm_step(&cpu, 1);
        assert_eq("STRD [sp,#28]: first word", 0x5BE0CD19u,
                  arm_read32(&cpu, sp + 28));
        assert_eq("STRD [sp,#28]: second word", 0x1F83D9ABu,
                  arm_read32(&cpu, sp + 32));
        arm_cpu_destroy(&cpu);
    }
}

/* ===================================================================
 * ARMv8-M TrustZone-M: SAU + IDAU security attribution (Phase 1)
 * =================================================================== */
static void tz_add_region(arm_cpu_t *cpu, unsigned r, uint32_t base,
                          uint32_t limit, bool nsc) {
    cpu->sau_rbar[r] = base & ~0x1fu;
    cpu->sau_rlar[r] = (limit & ~0x1fu) | ARM_SAU_RLAR_ENABLE
                       | (nsc ? ARM_SAU_RLAR_NSC : 0);
}

static void test_trustzone_sau(void) {
    if (verbose) printf("--- ARMv8-M TrustZone SAU/IDAU attribution tests ---\n");
    arm_cpu_t cpu;
    setup_arm(&cpu);           /* cc2538: tz_enabled == false */

    /* Non-TZ SoC: attribution is always Non-secure regardless of SAU state. */
    assert_eq("no-TZ SoC => NONSECURE", ARM_SEC_NONSECURE,
              arm_security_attr(&cpu, 0x20000000));

    /* Enable the extension for the engine tests. */
    cpu.tz_enabled = true;
    cpu.sau_sregions = 8;

    /* SAU disabled, ALLNS=0 => entire space Secure. */
    cpu.sau_ctrl = 0;
    assert_eq("SAU off, ALLNS=0 => SECURE", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0x00000000));
    assert_eq("SAU off, ALLNS=0 => SECURE (high)", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0xE000ED00));

    /* SAU disabled, ALLNS=1 => entire space Non-secure. */
    cpu.sau_ctrl = ARM_SAU_CTRL_ALLNS;
    assert_eq("SAU off, ALLNS=1 => NONSECURE", ARM_SEC_NONSECURE,
              arm_security_attr(&cpu, 0x20000000));

    /* SAU enabled with one Non-secure region [0x20000000, 0x2000FFFF]. */
    memset(cpu.sau_rbar, 0, sizeof(cpu.sau_rbar));
    memset(cpu.sau_rlar, 0, sizeof(cpu.sau_rlar));
    cpu.sau_ctrl = ARM_SAU_CTRL_ENABLE;
    tz_add_region(&cpu, 0, 0x20000000, 0x2000FFFF, false);
    assert_eq("in NS region => NONSECURE", ARM_SEC_NONSECURE,
              arm_security_attr(&cpu, 0x20008000));
    assert_eq("region base is inclusive", ARM_SEC_NONSECURE,
              arm_security_attr(&cpu, 0x20000000));
    assert_eq("region limit low bits set (inclusive)", ARM_SEC_NONSECURE,
              arm_security_attr(&cpu, 0x2000FFFF));
    assert_eq("just below region => SECURE", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0x1FFFFFFF));
    assert_eq("just above region => SECURE", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0x20010000));

    /* NSC region => Secure, callable. */
    tz_add_region(&cpu, 1, 0x00040000, 0x00040FFF, true);
    assert_eq("in NSC region => NSC", ARM_SEC_NSC,
              arm_security_attr(&cpu, 0x00040100));
    assert_eq("outside all regions => SECURE", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0x00080000));

    /* Overlapping regions => Secure (invalid), even if both are NS. */
    tz_add_region(&cpu, 2, 0x30000000, 0x3000FFFF, false);
    tz_add_region(&cpu, 3, 0x30008000, 0x3001FFFF, false);
    assert_eq("overlapping NS regions => SECURE", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0x3000C000));
    assert_eq("non-overlapping part of region 2 => NONSECURE", ARM_SEC_NONSECURE,
              arm_security_attr(&cpu, 0x30001000));

    /* Disabled region is ignored. */
    cpu.sau_rlar[0] &= ~ARM_SAU_RLAR_ENABLE;
    assert_eq("disabled region => SECURE", ARM_SEC_SECURE,
              arm_security_attr(&cpu, 0x20008000));
}

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
    test_umull();
    test_subs_rd_eq_rn();
    test_vfp_double();
    test_ldrd_strd();
    test_bit_field_ops();
    test_ldaex_stlex();
    test_m4_dsp_halfword_multiply();
    test_m4_vfp();
    test_anti_replay_ops();
    test_trustzone_sau();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
