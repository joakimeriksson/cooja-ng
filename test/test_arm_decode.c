/*
 * arm-decode — differential test: decoder + reference model vs the interpreter.
 *
 * WHY DIFFERENTIAL, AND WHY EXHAUSTIVE
 *
 * src/arm/arm_decode.c and arm_execute_decoded() together are a *second*
 * implementation of a slice of the Thumb-16 semantics — the slice the JIT is
 * allowed to compile.  Two implementations of the same thing drift, and when
 * the drift is in a flag rather than a result it stays invisible until some
 * later conditional branch goes the wrong way.  That is exactly how the
 * MSP430 JIT's one shipped bug (ADDC leaving V stale) behaved, and why it was
 * only reachable once a block had gone hot.
 *
 * So this does not inspect the decoder; it *runs* both paths:
 *
 *   for every one of the 65536 Thumb halfwords
 *     if the decoder claims to support it
 *       for several pseudo-random register/flag states
 *         A: execute the real encoding through arm_step()
 *         B: execute the decoded form through arm_execute_decoded()
 *         require r0..r15, xPSR and the cycle count to match exactly
 *
 * Exhaustive over encodings is what makes this worth having: it catches an
 * encoding the decoder claims but mis-parses, which a hand-written case list
 * would simply never mention.  The random states catch flag rules that happen
 * to agree on zero.
 *
 * A failure here means the JIT would have generated wrong code, and is a hard
 * stop — never a "close enough".
 */
#include "arm_cpu.h"
#include "arm_config.h"
#include "arm_decode.h"

#include <stdio.h>
#include <string.h>

#define CODE_BASE (ARM_FLASH_BASE + 0x100)

static int passed, failed;
static int verbose;

/* Deterministic PRNG so a failure is reproducible from the seed alone. */
static uint32_t rng_state = 0x13579BDFu;
static uint32_t rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static void write_flash32(arm_cpu_t *cpu, uint32_t addr, uint32_t val) {
    uint32_t off = addr - ARM_FLASH_BASE;
    cpu->flash[off]     = val & 0xFF;
    cpu->flash[off + 1] = (val >> 8) & 0xFF;
    cpu->flash[off + 2] = (val >> 16) & 0xFF;
    cpu->flash[off + 3] = (val >> 24) & 0xFF;
}

static void setup(arm_cpu_t *cpu, uint16_t hw) {
    arm_cpu_init(cpu, &cc2538_config);
    write_flash32(cpu, ARM_FLASH_BASE, ARM_SRAM_BASE + ARM_SRAM_SIZE);
    write_flash32(cpu, ARM_FLASH_BASE + 4, CODE_BASE + 1);
    arm_cpu_reset(cpu);
    uint32_t off = CODE_BASE - ARM_FLASH_BASE;
    cpu->flash[off]     = hw & 0xFF;
    cpu->flash[off + 1] = (hw >> 8) & 0xFF;
}

/* Interesting values: the flag edges are where two flag rules disagree. */
static uint32_t interesting(int i) {
    static const uint32_t v[] = {
        0u, 1u, 2u, 0x7FFFFFFFu, 0x80000000u, 0x80000001u,
        0xFFFFFFFFu, 0xFFFFFFFEu, 0x0000FFFFu, 0xFFFF0000u,
    };
    return v[i % (int)(sizeof(v) / sizeof(v[0]))];
}

static int compare(uint16_t hw, const arm_decoded_insn_t *di,
                   const arm_cpu_t *a, const arm_cpu_t *b, int trial) {
    int bad = 0;
    for (int r = 0; r <= 15; r++) {
        if (a->reg[r] != b->reg[r]) {
            printf("  FAIL: hw=0x%04x trial=%d  r%d: interp=0x%08x model=0x%08x\n",
                   hw, trial, r, a->reg[r], b->reg[r]);
            bad = 1;
        }
    }
    /* Compare the APSR flags only — the rest of xPSR (IPSR, IT bits) is not
     * something this subset touches. */
    uint32_t fa = a->xpsr & 0xF0000000u, fb = b->xpsr & 0xF0000000u;
    if (fa != fb) {
        printf("  FAIL: hw=0x%04x trial=%d  APSR: interp=N%dZ%dC%dV%d "
               "model=N%dZ%dC%dV%d\n", hw, trial,
               !!(fa & 0x80000000u), !!(fa & 0x40000000u),
               !!(fa & 0x20000000u), !!(fa & 0x10000000u),
               !!(fb & 0x80000000u), !!(fb & 0x40000000u),
               !!(fb & 0x20000000u), !!(fb & 0x10000000u));
        bad = 1;
    }
    if (a->cycles != b->cycles) {
        printf("  FAIL: hw=0x%04x trial=%d  cycles: interp=%lld model=%lld\n",
               hw, trial, (long long)a->cycles, (long long)b->cycles);
        bad = 1;
    }
    if (bad && verbose)
        printf("        decoded: klass=%d op=%d rd=%u rn=%u rm=%u imm=0x%x\n",
               (int)di->klass, (int)di->op, di->rd, di->rn, di->rm, di->imm);
    return bad;
}

#define TRIALS 6

int run_arm_decode_tests(int verbose_flag);

int run_arm_decode_tests(int verbose_flag) {
    verbose = verbose_flag;
    passed = failed = 0;

    printf("=== ARM decoder differential tests ===\n");
    printf("Exhaustive over all 65536 Thumb halfwords, %d random states each;\n"
           "decoder + arm_execute_decoded() vs arm_step().\n\n", TRIALS);

    /* One CPU pair reused across the sweep — arm_cpu_init/destroy per encoding
     * would dominate the runtime. */
    int supported = 0, rejected = 0;

    for (uint32_t h = 0; h < 0x10000u; h++) {
        uint16_t hw = (uint16_t)h;

        /* Decode straight out of a scratch image so the decoder sees exactly
         * the bytes the interpreter will fetch. */
        uint8_t img[4] = { (uint8_t)(hw & 0xFF), (uint8_t)(hw >> 8), 0, 0 };
        arm_decoded_insn_t di;
        arm_decode_insn(img, CODE_BASE, sizeof(img), CODE_BASE, &di);

        if (di.klass == ARM_DEC_UNSUPPORTED) { rejected++; continue; }
        supported++;

        for (int t = 0; t < TRIALS; t++) {
            arm_cpu_t a, b;
            setup(&a, hw);
            setup(&b, hw);

            /* Identical starting state, including the incoming flags — a
             * flag rule that only clears bits looks correct if they start 0. */
            for (int r = 0; r <= 12; r++) {
                uint32_t v = (t < 4) ? interesting(r + t) : rnd();
                a.reg[r] = b.reg[r] = v;
            }
            uint32_t flags = (rnd() & 0xF0000000u);
            a.xpsr = (a.xpsr & ~0xF0000000u) | flags;
            b.xpsr = (b.xpsr & ~0xF0000000u) | flags;

            /* A: the interpreter. */
            arm_step(&a, 1);

            /* B: the model.  arm_step charges the cycle and pre-advances PC
             * centrally before dispatch, so do the same here. */
            b.reg[ARM_PC] = CODE_BASE + 2;
            b.cycles += 1;
            b.instructions++;
            if (!arm_execute_decoded(&b, &di)) {
                printf("  FAIL: hw=0x%04x decoded as klass=%d but the model "
                       "refused to execute it\n", hw, (int)di.klass);
                failed++;
                arm_cpu_destroy(&a); arm_cpu_destroy(&b);
                break;
            }

            if (compare(hw, &di, &a, &b, t)) failed++;
            else passed++;

            arm_cpu_destroy(&a);
            arm_cpu_destroy(&b);
        }
    }

    printf("  encodings: %d supported, %d rejected (rejected are interpreted, "
           "never compiled)\n", supported, rejected);
    printf("\n--- Results: %d passed, %d failed ---\n", passed, failed);
    return failed;
}
