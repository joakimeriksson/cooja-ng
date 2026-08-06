/*
 * arm_jit — GNU Lightning code generation for hot ARM basic blocks.
 * See include/arm/arm_jit.h for the safety argument and why verification is
 * lockstep state comparison rather than "the tests still pass".
 *
 * Register allocation (mirrors src/msp430/msp430_jit.c so the two read alike):
 *   JIT_V0 = arm_cpu_t *cpu          (callee-saved, live for the block)
 *   JIT_V1 = &cpu->reg[0]            (callee-saved)
 *   JIT_V2 = cpu->xpsr working copy  (callee-saved; loaded at entry, stored
 *                                     once at exit — every flag-setting
 *                                     instruction touches it, so keeping it
 *                                     in a register is most of the win)
 *   JIT_R0, JIT_R1, JIT_R2 = temporaries
 *
 * Three things the interpreter does per instruction that a compiled block
 * does ONCE, at the end.  They are what make this worth doing, and each is
 * only legal because of a property of the decoder's subset:
 *
 *   - PC write.  No instruction in the subset reads PC and none can fault, so
 *     PC only has to be architecturally correct on block exit.  (Operands are
 *     r0-r7, never r15 — the decoder enforces it.)
 *   - Cycle accounting.  arm_step charges exactly 1 cycle per Thumb-16
 *     instruction, centrally, so a block of N costs exactly N.
 *   - xPSR store.  Flags live in JIT_V2 across the whole block.
 *
 * Operands are loaded zero-extended (jit_ldxi_ui) so 64-bit host arithmetic
 * produces the carry for free in bit 32 — exactly how set_add_flags() reads it
 * in C.  ASR is the one exception and loads sign-extended.
 */
#ifdef HAVE_LIGHTNING

#include "arm_jit.h"
#include "arm_cpu.h"
#include "arm_decode.h"

#include <lightning.h>
#include <stddef.h>
#include <stdlib.h>

/* Blocks shorter than this don't repay the call into native code.  Runtime
 * settable so `arm-jit` can compile one-instruction blocks and verify the
 * generated code for every supported encoding. */
static int arm_jit_min_block(void) {
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CSIM_ARM_JIT_MIN_BLOCK");
        v = e ? atoi(e) : 4;
        if (v < 1) v = 1;
    }
    return v;
}

#define OFF_REG    offsetof(arm_cpu_t, reg)
#define OFF_XPSR   offsetof(arm_cpu_t, xpsr)
#define OFF_CYCLES offsetof(arm_cpu_t, cycles)
#define OFF_INSTR  offsetof(arm_cpu_t, instructions)
#define OFF_ITER   offsetof(arm_cpu_t, jit_iter_budget)

/* APSR bits, and the masks that clear the ones a given form writes. */
#define F_N        0x80000000u
#define F_Z        0x40000000u
#define F_C        0x20000000u
#define F_V        0x10000000u
#define CLR_NZ     0x3FFFFFFFu
#define CLR_NZC    0x1FFFFFFFu
#define CLR_NZCV   0x0FFFFFFFu
#define LOW32      0xFFFFFFFFu

#define LOADREG_U(t, r) jit_ldxi_ui(t, JIT_V1, (jit_word_t)((r) * 4))
#define LOADREG_I(t, r) jit_ldxi_i(t,  JIT_V1, (jit_word_t)((r) * 4))
#define STOREREG(t, r)  jit_stxi_i((jit_word_t)((r) * 4), JIT_V1, t)

void csim_lightning_init(void) {
    static int inited;
    if (!inited) { init_jit(NULL); inited = 1; }
}

/* N and Z from a 32-bit result already in `res`.  The caller has cleared the
 * bits it writes, so this only ORs. */
static void emit_nz_bits(jit_state_t *_jit, int res) {
    jit_node_t *nz = jit_bnei(res, 0);
    jit_ori(JIT_V2, JIT_V2, F_Z);
    jit_patch(nz);
    jit_node_t *nn = jit_bmci(res, F_N);
    jit_ori(JIT_V2, JIT_V2, F_N);
    jit_patch(nn);
}

/* Logical ops: clear N/Z, compute, set N/Z.  C and V are untouched, matching
 * set_nz() in arm_cpu.c. */
static void emit_logic_nz(jit_state_t *_jit, int res) {
    jit_andi(JIT_V2, JIT_V2, CLR_NZ);
    emit_nz_bits(_jit, res);
}

/*
 * ADD/CMN — mirrors set_add_flags(a, b, (uint64)a + b):
 *   C = result64 > 0xFFFFFFFF
 *   V = ((a ^ r32) & (b ^ r32)) >> 31
 * On entry R0 = a, R1 = b (both zero-extended).  Leaves r32 in R2 and
 * destroys R0/R1 (safe: the result is in R2 by then).
 */
static void emit_add_flags(jit_state_t *_jit) {
    jit_andi(JIT_V2, JIT_V2, CLR_NZCV);
    jit_addr(JIT_R2, JIT_R0, JIT_R1);              /* 64-bit sum */
    jit_node_t *nc = jit_blei_u(JIT_R2, (jit_word_t)LOW32);
    jit_ori(JIT_V2, JIT_V2, F_C);
    jit_patch(nc);
    jit_andi(JIT_R2, JIT_R2, (jit_word_t)LOW32);   /* r32 */

    jit_xorr(JIT_R0, JIT_R0, JIT_R2);              /* a ^ r */
    jit_xorr(JIT_R1, JIT_R1, JIT_R2);              /* b ^ r */
    jit_andr(JIT_R0, JIT_R0, JIT_R1);
    jit_node_t *nv = jit_bmci(JIT_R0, F_N);
    jit_ori(JIT_V2, JIT_V2, F_V);
    jit_patch(nv);

    emit_nz_bits(_jit, JIT_R2);
}

/*
 * SUB/CMP/RSB — mirrors set_sub_flags(a, b, (uint64)a - b):
 *   C = a >= b   (unsigned; "no borrow")
 *   V = ((a ^ b) & (a ^ r32)) >> 31
 * On entry R0 = a, R1 = b.  Leaves r32 in R2, destroys R0/R1.
 */
static void emit_sub_flags(jit_state_t *_jit) {
    jit_andi(JIT_V2, JIT_V2, CLR_NZCV);
    jit_node_t *nc = jit_bltr_u(JIT_R0, JIT_R1);   /* a < b -> no carry */
    jit_ori(JIT_V2, JIT_V2, F_C);
    jit_patch(nc);

    jit_subr(JIT_R2, JIT_R0, JIT_R1);
    jit_andi(JIT_R2, JIT_R2, (jit_word_t)LOW32);   /* r32 */

    jit_xorr(JIT_R1, JIT_R0, JIT_R1);              /* a ^ b */
    jit_xorr(JIT_R0, JIT_R0, JIT_R2);              /* a ^ r */
    jit_andr(JIT_R0, JIT_R0, JIT_R1);
    jit_node_t *nv = jit_bmci(JIT_R0, F_N);
    jit_ori(JIT_V2, JIT_V2, F_V);
    jit_patch(nv);

    emit_nz_bits(_jit, JIT_R2);
}

/*
 * Materialise an ARM condition code into JIT_R0 as 0/1, from the flags held in
 * JIT_V2.  Mirrors condition_passed() in arm_cpu.c case for case — the
 * highest-risk function in this file, since a wrong condition silently takes
 * the wrong branch.  Destroys R0/R1/R2, so it is only emitted as a terminator.
 */
static void emit_cond(jit_state_t *_jit, int cond) {
    switch (cond >> 1) {
    case 0: jit_andi(JIT_R0, JIT_V2, F_Z); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 1: jit_andi(JIT_R0, JIT_V2, F_C); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 2: jit_andi(JIT_R0, JIT_V2, F_N); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 3: jit_andi(JIT_R0, JIT_V2, F_V); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 4:                                       /* HI/LS: C && !Z */
        jit_andi(JIT_R0, JIT_V2, F_C); jit_nei(JIT_R0, JIT_R0, 0);
        jit_andi(JIT_R1, JIT_V2, F_Z); jit_eqi(JIT_R1, JIT_R1, 0);
        jit_andr(JIT_R0, JIT_R0, JIT_R1);
        break;
    case 5:                                       /* GE/LT: N == V */
        jit_andi(JIT_R0, JIT_V2, F_N); jit_nei(JIT_R0, JIT_R0, 0);
        jit_andi(JIT_R1, JIT_V2, F_V); jit_nei(JIT_R1, JIT_R1, 0);
        jit_eqr(JIT_R0, JIT_R0, JIT_R1);
        break;
    case 6:                                       /* GT/LE: !Z && N == V */
        jit_andi(JIT_R0, JIT_V2, F_N); jit_nei(JIT_R0, JIT_R0, 0);
        jit_andi(JIT_R1, JIT_V2, F_V); jit_nei(JIT_R1, JIT_R1, 0);
        jit_eqr(JIT_R0, JIT_R0, JIT_R1);
        jit_andi(JIT_R2, JIT_V2, F_Z); jit_eqi(JIT_R2, JIT_R2, 0);
        jit_andr(JIT_R0, JIT_R0, JIT_R2);
        break;
    default: jit_movi(JIT_R0, 1); break;          /* AL */
    }
    if (cond & 1) jit_eqi(JIT_R0, JIT_R0, 0);     /* invert the odd codes */
}

/* Emit one decoded instruction.  Returns 0 if it cannot be emitted (which
 * makes the whole block be refused — never silently skipped). */
static int emit_insn(jit_state_t *_jit, const arm_decoded_insn_t *di) {
    switch (di->klass) {

    case ARM_DEC_B_UNCOND:
    case ARM_DEC_B_COND:
        /* Terminators are handled by the epilogue, which is the only place
         * that knows both the branch target and the fall-through address. */
        return 1;

    case ARM_DEC_SHIFT_IMM: {
        int n = (int)di->imm;                      /* 1..31, decoder-checked */
        if (n < 1 || n > 31) return 0;
        jit_andi(JIT_V2, JIT_V2, CLR_NZC);
        if (di->shift == ARM_SH_ASR) {
            LOADREG_I(JIT_R0, di->rm);             /* sign-extended */
            jit_rshi(JIT_R1, JIT_R0, n - 1);
            jit_node_t *nc = jit_bmci(JIT_R1, 1);
            jit_ori(JIT_V2, JIT_V2, F_C);
            jit_patch(nc);
            jit_rshi(JIT_R2, JIT_R0, n);
        } else {
            LOADREG_U(JIT_R0, di->rm);             /* zero-extended */
            if (di->shift == ARM_SH_LSL) {
                jit_rshi_u(JIT_R1, JIT_R0, 32 - n);
                jit_node_t *nc = jit_bmci(JIT_R1, 1);
                jit_ori(JIT_V2, JIT_V2, F_C);
                jit_patch(nc);
                jit_lshi(JIT_R2, JIT_R0, n);
            } else {                               /* LSR */
                jit_rshi_u(JIT_R1, JIT_R0, n - 1);
                jit_node_t *nc = jit_bmci(JIT_R1, 1);
                jit_ori(JIT_V2, JIT_V2, F_C);
                jit_patch(nc);
                jit_rshi_u(JIT_R2, JIT_R0, n);
            }
        }
        jit_andi(JIT_R2, JIT_R2, (jit_word_t)LOW32);
        emit_nz_bits(_jit, JIT_R2);
        STOREREG(JIT_R2, di->rd);
        return 1;
    }

    case ARM_DEC_ALU_REG:
    case ARM_DEC_ALU_IMM: {
        int is_imm = (di->klass == ARM_DEC_ALU_IMM);

        /* R0 = operand a (reg[rn]), R1 = operand b (reg[rm] or #imm) */
        LOADREG_U(JIT_R0, di->rn);
        if (is_imm) jit_movi(JIT_R1, (jit_word_t)di->imm);
        else        LOADREG_U(JIT_R1, di->rm);

        switch (di->op) {
        case ARM_ALU_ADD:
        case ARM_ALU_CMN:
            emit_add_flags(_jit);
            break;

        case ARM_ALU_SUB:
        case ARM_ALU_CMP:
            emit_sub_flags(_jit);
            break;

        case ARM_ALU_RSB:
            /* NEG Rd, Rm: 0 - reg[rn], flags taken against a == 0 (the
             * interpreter's t16_dp_spec case 0x9 does exactly this). */
            jit_movr(JIT_R1, JIT_R0);
            jit_movi(JIT_R0, 0);
            emit_sub_flags(_jit);
            break;

        case ARM_ALU_AND:
        case ARM_ALU_TST:
            jit_andr(JIT_R2, JIT_R0, JIT_R1);
            emit_logic_nz(_jit, JIT_R2);
            break;

        case ARM_ALU_EOR:
            jit_xorr(JIT_R2, JIT_R0, JIT_R1);
            emit_logic_nz(_jit, JIT_R2);
            break;

        case ARM_ALU_ORR:
            jit_orr(JIT_R2, JIT_R0, JIT_R1);
            emit_logic_nz(_jit, JIT_R2);
            break;

        case ARM_ALU_BIC:
            /* a & ~b: a is zero-extended, so the complement's high bits are
             * masked off by the AND — no explicit truncation needed. */
            jit_comr(JIT_R1, JIT_R1);
            jit_andr(JIT_R2, JIT_R0, JIT_R1);
            emit_logic_nz(_jit, JIT_R2);
            break;

        case ARM_ALU_MOV:
            if (is_imm) jit_movr(JIT_R2, JIT_R1);
            else        jit_movr(JIT_R2, JIT_R0);
            emit_logic_nz(_jit, JIT_R2);
            break;

        case ARM_ALU_MVN:
            jit_comr(JIT_R2, JIT_R0);
            jit_andi(JIT_R2, JIT_R2, (jit_word_t)LOW32);
            emit_logic_nz(_jit, JIT_R2);
            break;

        default:
            return 0;
        }

        if (di->writes_result) STOREREG(JIT_R2, di->rd);
        return 1;
    }

    default:
        return 0;
    }
}

arm_compiled_block_t *arm_jit_compile(const arm_basic_block_t *block,
                                      arm_cpu_t *cpu) {
    (void)cpu;
    if (!block || block->length < 1) return NULL;

    /* insns[0..length-1] are decoder-supported by construction (the block
     * decoder stops *before* the first unsupported instruction), so a block
     * that stopped early is still safe: it covers the supported prefix and
     * exits with PC = end_pc for the interpreter to continue from. */
    const arm_decoded_insn_t *last = &block->insns[block->length - 1];
    int term_is_branch = (last->klass == ARM_DEC_B_UNCOND ||
                          last->klass == ARM_DEC_B_COND);

    /*
     * A conditional branch back to the block's own first instruction is a
     * loop, and compiling it as a native loop rather than one call per
     * iteration is where the real win is: embedded firmware spends most of
     * its time in exactly this shape (delay loops, polling loops, memcpy).
     *
     * It cannot be allowed to spin unbounded, though — the block contains no
     * event check, so running N iterations advances `cycles` by N*length and
     * would sail past a scheduled peripheral event.  The dispatcher therefore
     * hands in an iteration budget (cpu->jit_iter_budget) computed from both
     * the instruction budget and the time to the next event; the loop
     * decrements it and exits when it runs out, leaving PC at the loop top so
     * the next entry resumes cleanly.
     */
    int is_loop = (block->length >= 2) &&
                  (last->klass == ARM_DEC_B_COND) &&
                  (last->imm == block->start_pc);

    /* Minimum size is per-kind, not global.  A straight-line block has to
     * repay one call per execution, so 4 instructions is the floor.  A loop
     * block repays that call across every iteration it runs — the dominant
     * shape in firmware is a 2-instruction delay loop (SUBS/BNE was 99.8% of
     * executed instructions in zephyr-synchronization), and holding it to the
     * straight-line minimum would exclude exactly the case worth compiling. */
    int min_len = is_loop ? 2 : arm_jit_min_block();
    if (block->length < min_len) return NULL;

    jit_state_t *_jit = jit_new_state();
    if (!_jit) return NULL;

    jit_prolog();
    jit_node_t *arg = jit_arg();
    jit_getarg(JIT_V0, arg);
    jit_addi(JIT_V1, JIT_V0, OFF_REG);
    jit_ldxi_ui(JIT_V2, JIT_V0, OFF_XPSR);

    jit_node_t *loop_top = NULL, *exhausted = NULL;
    if (is_loop) {
        loop_top = jit_label();
        jit_ldxi_i(JIT_R0, JIT_V0, OFF_ITER);
        exhausted = jit_beqi(JIT_R0, 0);
        jit_subi(JIT_R0, JIT_R0, 1);
        jit_stxi_i(OFF_ITER, JIT_V0, JIT_R0);
    }

    int body = term_is_branch ? block->length - 1 : block->length;
    for (int i = 0; i < body; i++) {
        if (!emit_insn(_jit, &block->insns[i])) {
            jit_destroy_state();
            return NULL;
        }
    }

    /* --- terminator + final PC --- */
    jit_node_t *joins[2]; int njoin = 0;

    if (is_loop) {
        emit_cond(_jit, last->cond);
        jit_node_t *fell_through = jit_beqi(JIT_R0, 0);
        jit_patch_at(jit_jmpi(), loop_top);          /* taken -> next iteration */
        jit_patch(fell_through);
        jit_movi(JIT_R0, (jit_word_t)block->end_pc); /* loop condition failed */
        STOREREG(JIT_R0, ARM_PC);
        joins[njoin++] = jit_jmpi();
        jit_patch(exhausted);                        /* budget spent */
        jit_movi(JIT_R0, (jit_word_t)block->start_pc);
        STOREREG(JIT_R0, ARM_PC);
    } else if (last->klass == ARM_DEC_B_COND) {
        emit_cond(_jit, last->cond);
        jit_node_t *not_taken = jit_beqi(JIT_R0, 0);
        jit_movi(JIT_R0, (jit_word_t)last->imm);
        STOREREG(JIT_R0, ARM_PC);
        joins[njoin++] = jit_jmpi();
        jit_patch(not_taken);
        jit_movi(JIT_R0, (jit_word_t)block->end_pc);
        STOREREG(JIT_R0, ARM_PC);
    } else {
        uint32_t final_pc = (last->klass == ARM_DEC_B_UNCOND) ? last->imm
                                                              : block->end_pc;
        jit_movi(JIT_R0, (jit_word_t)final_pc);
        STOREREG(JIT_R0, ARM_PC);
    }
    for (int i = 0; i < njoin; i++) jit_patch(joins[i]);

    /* Flags back to memory.  Cycle and instruction accounting is the
     * dispatcher's job — it is the only side that knows how many iterations a
     * loop block actually ran. */
    jit_stxi_i(OFF_XPSR, JIT_V0, JIT_V2);
    jit_reti(block->length);
    jit_epilog();

    arm_compiled_fn fn = (arm_compiled_fn)jit_emit();
    if (!fn) { jit_destroy_state(); return NULL; }
    jit_clear_state();                 /* frees the IR, keeps the code */

    arm_compiled_block_t *cb =
        (arm_compiled_block_t *)malloc(sizeof(arm_compiled_block_t));
    if (!cb) { jit_destroy_state(); return NULL; }
    cb->fn        = fn;
    cb->length    = block->length;
    cb->start_pc  = block->start_pc;
    cb->end_pc    = block->end_pc;
    cb->is_loop   = is_loop;
    cb->jit_state = _jit;
    return cb;
}

void arm_jit_free(arm_compiled_block_t *cb) {
    if (!cb) return;
    if (cb->jit_state) {
        jit_state_t *_jit = cb->jit_state;
        jit_destroy_state();
    }
    free(cb);
}

#endif /* HAVE_LIGHTNING */
