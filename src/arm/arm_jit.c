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

/*
 * Minimum block length worth compiling.
 *
 * The obvious reasoning — "a block has to be long enough to repay the call
 * into native code, so require 4" — is wrong, and measurably so.  Raising this
 * from 1 to 4 does not trade a little coverage for a little call overhead; it
 * falls off a cliff:
 *
 *   zephyr-synchronization, 60 s sim      min=4        min=1
 *     instructions through compiled code   26.3%       98.3%
 *     wall clock                           1.55 s      0.58 s
 *
 * The cause is that the fallback is all-or-nothing *per arm_step call*.  When
 * the dispatcher finds no block at the current PC it hands the interpreter the
 * caller's entire remaining budget (which it must — see ARM_JIT_BATCH), so one
 * uncompilable PC costs every compilable block that would have followed it in
 * that call.  Refusing a 1-instruction block at the PC right after a Thumb-2
 * instruction therefore does not cost one instruction, it costs the rest of
 * the slice.  Compiling everything keeps the dispatcher chaining block to
 * block and it never falls back at all.
 *
 * The cost of doing that is small and was measured rather than assumed: 163
 * live compiled blocks and +4.8 MB peak RSS on the run above.  The working set
 * of a firmware image is nothing like the 65536-slot cache.
 *
 * Still runtime-settable, both because it is how the cliff above was found and
 * so `arm-jit` can pin it while verifying generated code encoding by encoding.
 */
static int arm_jit_min_block(void) {
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CSIM_ARM_JIT_MIN_BLOCK");
        v = e ? atoi(e) : 1;
        if (v < 1) v = 1;
    }
    return v;
}

#define OFF_REG    offsetof(arm_cpu_t, reg)
#define OFF_XPSR   offsetof(arm_cpu_t, xpsr)
#define OFF_CYCLES offsetof(arm_cpu_t, cycles)
#define OFF_INSTR  offsetof(arm_cpu_t, instructions)
#define OFF_ITER   offsetof(arm_cpu_t, jit_iter_budget)
#define OFF_PART   offsetof(arm_cpu_t, jit_partial)
#define OFF_SRAM   offsetof(arm_cpu_t, sram)
#define OFF_FLASH  offsetof(arm_cpu_t, flash)

/*
 * The inline memory path reads and writes the emulated SRAM byte array with
 * native host loads and stores, which is only equivalent to the interpreter's
 * byte-at-a-time assembly on a little-endian host.  Rather than emit
 * byte-shuffling code nobody would ever exercise, memory ops are simply not
 * compiled on a big-endian host — they side-exit to the interpreter, which is
 * already correct there.
 */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#  define ARM_JIT_INLINE_MEM 0
#else
#  define ARM_JIT_INLINE_MEM 1
#endif

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
 * N as 0 or 1, from JIT_V2 bit 31.
 *
 * NEVER write this as `jit_andi(dst, JIT_V2, F_N)`.  **GNU Lightning 2.2.3 on
 * x86-64 miscompiles `jit_andi` when the immediate is exactly 0x80000000: the
 * result is always 0.**  Isolated in a standalone Lightning program, so it is
 * the backend and not this file; `ori`, `bmci`, `bmsi` and `andi` with
 * 0xFFFFFFFF are all correct, and the ARM64 backend is correct throughout.
 *
 * It cost a silent wrong-branch bug: emit_cond() extracted N this way, so MI,
 * PL, GE, LT, GT and LE all behaved as though N were clear — on x86-64 only,
 * while `arm-decode` passed 269952/269952 on both hosts and the simulations
 * still produced plausible output.  A logical right shift has no immediate at
 * all, and hands back the 0/1 the condition logic wants directly.
 */
static void emit_n_bit(jit_state_t *_jit, int dst) {
    jit_rshi_u(dst, JIT_V2, 31);
    jit_andi(dst, dst, 1);       /* belt and braces: keep it a clean 0/1 */
}

/*
 * Materialise an ARM condition code into JIT_R0 as 0/1, from the flags held in
 * JIT_V2.  Mirrors condition_passed() in arm_cpu.c case for case — the
 * highest-risk function in this file, since a wrong condition silently takes
 * the wrong branch.  Destroys R0/R1/R2, so it is only emitted as a terminator.
 *
 * `arm-jit` verifies all 14 condition codes against the interpreter on
 * whatever host it is built for.  That suite exists because of the bug in
 * emit_n_bit()'s comment.
 */
static void emit_cond(jit_state_t *_jit, int cond) {
    switch (cond >> 1) {
    case 0: jit_andi(JIT_R0, JIT_V2, F_Z); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 1: jit_andi(JIT_R0, JIT_V2, F_C); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 2: emit_n_bit(_jit, JIT_R0); break;
    case 3: jit_andi(JIT_R0, JIT_V2, F_V); jit_nei(JIT_R0, JIT_R0, 0); break;
    case 4:                                       /* HI/LS: C && !Z */
        jit_andi(JIT_R0, JIT_V2, F_C); jit_nei(JIT_R0, JIT_R0, 0);
        jit_andi(JIT_R1, JIT_V2, F_Z); jit_eqi(JIT_R1, JIT_R1, 0);
        jit_andr(JIT_R0, JIT_R0, JIT_R1);
        break;
    case 5:                                       /* GE/LT: N == V */
        emit_n_bit(_jit, JIT_R0);
        jit_andi(JIT_R1, JIT_V2, F_V); jit_nei(JIT_R1, JIT_R1, 0);
        jit_eqr(JIT_R0, JIT_R0, JIT_R1);
        break;
    case 6:                                       /* GT/LE: !Z && N == V */
        emit_n_bit(_jit, JIT_R0);
        jit_andi(JIT_R1, JIT_V2, F_V); jit_nei(JIT_R1, JIT_R1, 0);
        jit_eqr(JIT_R0, JIT_R0, JIT_R1);
        jit_andi(JIT_R2, JIT_V2, F_Z); jit_eqi(JIT_R2, JIT_R2, 0);
        jit_andr(JIT_R0, JIT_R0, JIT_R2);
        break;
    default: jit_movi(JIT_R0, 1); break;          /* AL */
    }
    if (cond & 1) jit_eqi(JIT_R0, JIT_R0, 0);     /* invert the odd codes */
}

/*
 * A guard miss jumps here.  One stub per memory instruction, emitted after the
 * block body so the fast path is a straight fall-through with two
 * never-taken forward branches rather than two jumps over inline cold code.
 */
typedef struct {
    jit_node_t *from[2];    /* the guard branches that land here */
    int         nfrom;
    int         index;      /* instruction index within the block */
    uint32_t    pc;
} side_stub_t;

typedef struct {
    side_stub_t stubs[ARM_MAX_BLOCK_SIZE];
    int         nstubs;
    uint32_t    sram_base, sram_size;
    uint32_t    flash_base, flash_size;
} emit_ctx_t;

/* Register a guard branch as landing in instruction `idx`'s side-exit stub. */
static void stub_add(emit_ctx_t *cx, int idx, uint32_t pc, jit_node_t *from) {
    side_stub_t *st = cx->nstubs ? &cx->stubs[cx->nstubs - 1] : NULL;
    if (!st || st->index != idx) {
        st = &cx->stubs[cx->nstubs++];
        st->nfrom = 0; st->index = idx; st->pc = pc;
    }
    st->from[st->nfrom++] = from;
}

/*
 * Emit `R0 = reg[rn] + reg[rm] + imm`, truncated to 32 bits.
 * The truncation is not cosmetic: operands are loaded zero-extended into
 * 64-bit host registers, and ARM address arithmetic wraps at 2^32, so a base
 * near the top of the address space must wrap rather than run off into the
 * host's high half — where it would sail past the SRAM range check.
 */
static void emit_addr(jit_state_t *_jit, const arm_decoded_insn_t *di) {
    LOADREG_U(JIT_R0, di->rn);
    if (di->rm != ARM_DEC_NO_RM) {
        LOADREG_U(JIT_R1, di->rm);
        jit_addr(JIT_R0, JIT_R0, JIT_R1);
    }
    if (di->imm) jit_addi(JIT_R0, JIT_R0, (jit_word_t)di->imm);
    jit_andi(JIT_R0, JIT_R0, (jit_word_t)LOW32);
}

/*
 * LOAD / STORE.  Leaves R1 = byte offset into the SRAM array on the fast path.
 *
 * Two guards, both of which side-exit rather than fault:
 *   - alignment.  The interpreter splits an unaligned access into bytes via a
 *     different code path; reproducing that inline would be a second
 *     implementation of it, so unaligned accesses go back to the interpreter.
 *   - range.  `(uint32)(addr - sram_base) < sram_size` is the whole test, one
 *     subtract and one unsigned compare, and it excludes flash, ROM, the
 *     bit-band alias and every peripheral window in one go.  Everything it
 *     excludes is exactly what must not run inside a block.
 */
static int emit_mem(jit_state_t *_jit, emit_ctx_t *cx,
                    const arm_decoded_insn_t *di, int idx) {
    if (!ARM_JIT_INLINE_MEM) return 0;

    emit_addr(_jit, di);

    if (di->msize > 1) {
        jit_andi(JIT_R1, JIT_R0, (jit_word_t)(di->msize - 1));
        stub_add(cx, idx, di->pc, jit_bnei(JIT_R1, 0));
    }
    jit_subi(JIT_R1, JIT_R0, (jit_word_t)cx->sram_base);
    jit_andi(JIT_R1, JIT_R1, (jit_word_t)LOW32);
    stub_add(cx, idx, di->pc,
             jit_bgei_u(JIT_R1, (jit_word_t)(cx->sram_size - (di->msize - 1))));

    jit_ldxi(JIT_R2, JIT_V0, OFF_SRAM);           /* host SRAM base */

    if (di->klass == ARM_DEC_LOAD) {
        switch (di->msize) {
        case 1: if (di->sext) jit_ldxr_c (JIT_R0, JIT_R2, JIT_R1);
                else          jit_ldxr_uc(JIT_R0, JIT_R2, JIT_R1);
                break;
        case 2: if (di->sext) jit_ldxr_s (JIT_R0, JIT_R2, JIT_R1);
                else          jit_ldxr_us(JIT_R0, JIT_R2, JIT_R1);
                break;
        default: jit_ldxr_ui(JIT_R0, JIT_R2, JIT_R1); break;
        }
        STOREREG(JIT_R0, di->rd);
    } else {
        LOADREG_U(JIT_R0, di->rd);                /* the value to store */
        switch (di->msize) {
        case 1:  jit_stxr_c(JIT_R1, JIT_R2, JIT_R0); break;
        case 2:  jit_stxr_s(JIT_R1, JIT_R2, JIT_R0); break;
        default: jit_stxr_i(JIT_R1, JIT_R2, JIT_R0); break;
        }
    }
    return 1;
}

/*
 * LDR Rt, [PC, #imm] — the address is a compile-time constant, so the region
 * is resolved here and the load needs no guard at all.  Flash is read-only in
 * this model and the cache is flushed on reset (the only way the image can
 * change), so the resolution stays valid for the life of the block.
 *
 * The SRAM-before-flash order mirrors mem_read32(); the regions are disjoint
 * in every SoC csim models, but matching the interpreter's precedence costs
 * nothing and removes the question.
 */
static int emit_load_lit(jit_state_t *_jit, emit_ctx_t *cx,
                         const arm_decoded_insn_t *di) {
    uint32_t a = di->imm;
    jit_word_t region;
    uint32_t off;

    if (a >= cx->sram_base && a + 4 <= cx->sram_base + cx->sram_size) {
        region = OFF_SRAM;  off = a - cx->sram_base;
    } else if (a >= cx->flash_base && a + 4 <= cx->flash_base + cx->flash_size) {
        region = OFF_FLASH; off = a - cx->flash_base;
    } else {
        return 0;           /* literal outside RAM and flash — refuse */
    }
    jit_ldxi(JIT_R2, JIT_V0, region);
    jit_ldxi_ui(JIT_R0, JIT_R2, (jit_word_t)off);
    STOREREG(JIT_R0, di->rd);
    return 1;
}

/* Emit one decoded instruction.  Returns 0 if it cannot be emitted (which
 * makes the whole block be refused — never silently skipped). */
static int emit_insn(jit_state_t *_jit, emit_ctx_t *cx,
                     const arm_decoded_insn_t *di, int idx) {
    switch (di->klass) {

    case ARM_DEC_LOAD:
    case ARM_DEC_STORE:
        return emit_mem(_jit, cx, di, idx);

    case ARM_DEC_LOAD_LIT:
        return emit_load_lit(_jit, cx, di);

    case ARM_DEC_NOP:
        return 1;                      /* emit nothing; the cycle is counted
                                        * by the block's cyc_prefix table */

    case ARM_DEC_B_UNCOND:
    case ARM_DEC_B_COND:
    case ARM_DEC_CBZ:
        /* Terminators are handled by the epilogue, which is the only place
         * that knows both the branch target and the fall-through address. */
        return 1;

    case ARM_DEC_EXTEND:
        /* SXTB/SXTH/UXTB/UXTH — no flags.  Lightning's ext* forms carry no
         * immediate, which is one fewer way to meet the backend bug in
         * emit_n_bit()'s comment. */
        LOADREG_U(JIT_R1, di->rm);
        if (di->msize == 1) {
            if (di->sext) jit_extr_c(JIT_R0, JIT_R1);
            else          jit_extr_uc(JIT_R0, JIT_R1);
        } else {
            if (di->sext) jit_extr_s(JIT_R0, JIT_R1);
            else          jit_extr_us(JIT_R0, JIT_R1);
        }
        STOREREG(JIT_R0, di->rd);
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

        /*
         * The high-register ADD/MOV, ADR, ADD Rd,SP,#imm and ADD/SUB SP,#imm
         * compute a value and leave the flags alone.  They are the only forms
         * in the subset that do, and emitting the flag update anyway would be
         * a divergence no register comparison could see — it only shows up
         * when a later conditional branch reads a flag that should not have
         * moved.
         */
        if (!di->set_flags) {
            switch (di->op) {
            case ARM_ALU_ADD: jit_addr(JIT_R2, JIT_R0, JIT_R1); break;
            case ARM_ALU_SUB: jit_subr(JIT_R2, JIT_R0, JIT_R1); break;
            case ARM_ALU_MOV: jit_movr(JIT_R2, is_imm ? JIT_R1 : JIT_R0); break;
            default: return 0;
            }
            jit_andi(JIT_R2, JIT_R2, (jit_word_t)LOW32);
            if (di->writes_result) STOREREG(JIT_R2, di->rd);
            return 1;
        }

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
                          last->klass == ARM_DEC_B_COND ||
                          last->klass == ARM_DEC_CBZ);

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

    /*
     * Memory ops address the SRAM array as `sram[addr - sram_base]`, so a base
     * that is not word-aligned would make an aligned guest access land on an
     * unaligned host one.  Every SoC csim models puts SRAM at 0x20000000 or
     * similar; assert it rather than emit code that only works by luck.
     */
    emit_ctx_t cx;
    cx.nstubs     = 0;
    cx.sram_base  = cpu->sram_base;
    cx.sram_size  = cpu->sram_end - cpu->sram_base;
    cx.flash_base = cpu->flash_base;
    cx.flash_size = cpu->flash_end - cpu->flash_base;
    int mem_ok = ((cx.sram_base & 3u) == 0) && cx.sram_size >= 4;

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
        const arm_decoded_insn_t *di = &block->insns[i];
        int is_mem = (di->klass == ARM_DEC_LOAD || di->klass == ARM_DEC_STORE);
        if ((is_mem && !mem_ok) || !emit_insn(_jit, &cx, di, i)) {
            jit_destroy_state();
            return NULL;
        }
    }

    /* --- terminator + final PC --- */
    jit_node_t *joins[ARM_MAX_BLOCK_SIZE + 2]; int njoin = 0;

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
        if (cx.nstubs) joins[njoin++] = jit_jmpi();  /* jump over the stubs */
    } else if (last->klass == ARM_DEC_B_COND || last->klass == ARM_DEC_CBZ) {
        if (last->klass == ARM_DEC_CBZ) {
            /* CBZ/CBNZ tests a register against zero and never reads a flag.
             * `cond` carries 1 for CBNZ, 0 for CBZ. */
            LOADREG_U(JIT_R1, last->rn);
            if (last->cond) jit_nei(JIT_R0, JIT_R1, 0);
            else            jit_eqi(JIT_R0, JIT_R1, 0);
        } else {
            emit_cond(_jit, last->cond);
        }
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
    if (!is_loop && cx.nstubs) joins[njoin++] = jit_jmpi();

    /*
     * Side-exit stubs, all cold.  Each reports where the block stopped and how
     * much of it ran, then falls into the shared exit so the flag store
     * happens exactly once no matter which path got here.  A loop block hands
     * its iteration-budget decrement back, because the iteration it was in the
     * middle of did not complete — that keeps `iters_max - jit_iter_budget` a
     * count of *finished* iterations, which is what the cycle arithmetic
     * assumes.
     */
    for (int i = 0; i < cx.nstubs; i++) {
        side_stub_t *st = &cx.stubs[i];
        for (int j = 0; j < st->nfrom; j++) jit_patch(st->from[j]);
        jit_movi(JIT_R0, (jit_word_t)st->pc);
        STOREREG(JIT_R0, ARM_PC);
        jit_movi(JIT_R0, (jit_word_t)st->index);
        jit_stxi_i(OFF_PART, JIT_V0, JIT_R0);
        if (is_loop) {
            jit_ldxi_i(JIT_R0, JIT_V0, OFF_ITER);
            jit_addi(JIT_R0, JIT_R0, 1);
            jit_stxi_i(OFF_ITER, JIT_V0, JIT_R0);
        }
        if (i != cx.nstubs - 1) joins[njoin++] = jit_jmpi();
    }

    for (int i = 0; i < njoin; i++) jit_patch(joins[i]);

    /* Flags back to memory.  Cycle and instruction accounting is the
     * dispatcher's job — it is the only side that knows how many iterations a
     * loop block actually ran. */
    jit_stxi_i(OFF_XPSR, JIT_V0, JIT_V2);
    jit_reti(block->length);   /* unused — see the side-exit contract in the header */
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
    cb->has_store = 0;
    for (int i = 0; i < block->length; i++)
        if (block->insns[i].klass == ARM_DEC_STORE) { cb->has_store = 1; break; }
    cb->cyc_prefix[0] = 0;
    for (int i = 0; i < block->length; i++)
        cb->cyc_prefix[i + 1] =
            (uint16_t)(cb->cyc_prefix[i] + block->insns[i].cycles);
    cb->cycles_total = cb->cyc_prefix[block->length];
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
