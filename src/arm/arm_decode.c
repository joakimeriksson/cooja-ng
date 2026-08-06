/*
 * arm_decode — Thumb-16 decoder for the JIT-compilable subset.
 * See include/arm/arm_decode.h for why this exists and why the subset is small.
 *
 * The single hard requirement on this file: for every instruction it claims to
 * support, the fields it produces must describe *exactly* what arm_step's
 * handler does — same operands, same flag behaviour, same cycle cost.  Any
 * drift is a warm-block-only divergence, which is the failure mode that cost
 * the MSP430 JIT its one shipped bug.  `arm-decode` (test_arm_decode.c) checks
 * this by differential execution against the interpreter rather than by
 * inspection.
 *
 * Cycle model: arm_step charges 1 cycle per Thumb-16 instruction centrally,
 * before dispatch:
 *      if (top5 < 29) { cpu->reg[ARM_PC] = pc + 2; cpu->cycles += 1; }
 * and the load/store handlers then charge a second one themselves, so `cycles`
 * is 1 for the register-only forms and 2 for every memory access.  That is why
 * a block's cost is a sum over `cycles` and not simply its length — see
 * `cycles_total` / `cyc_prefix` in arm_jit.h.
 */
#include "arm_decode.h"

#include <string.h>

/* Mark `out` as not compilable and return the size so the caller can advance. */
static int reject(arm_decoded_insn_t *out, uint32_t pc, uint16_t hw) {
    memset(out, 0, sizeof(*out));
    out->pc    = pc;
    out->raw   = hw;
    out->size  = arm_is_thumb2(hw) ? 4 : 2;
    out->klass = ARM_DEC_UNSUPPORTED;
    return out->size;
}

static void base(arm_decoded_insn_t *out, uint32_t pc, uint16_t hw,
                 arm_dec_class_t k) {
    memset(out, 0, sizeof(*out));
    out->pc            = pc;
    out->raw           = hw;
    out->size          = 2;
    out->klass         = k;
    out->writes_result = true;
    out->cycles        = 1;
    out->rm            = ARM_DEC_NO_RM;
}

/*
 * Common tail for the memory forms.  `rn == ARM_SP` is allowed here even
 * though every other class is restricted to r0-r7: the [SP, #imm] encodings
 * are 31.4% of executed instructions on cc2538 RPL-UDP, so excluding them
 * would leave most of the win on the table for no safety gain — SP is an
 * ordinary base register to a load, not a PC-like special case.
 */
static void memop(arm_decoded_insn_t *out, uint32_t pc, uint16_t hw,
                arm_dec_class_t k, int rt, int rn, int msize) {
    base(out, pc, hw, k);
    out->rd            = (uint8_t)rt;
    out->rn            = (uint8_t)rn;
    out->msize         = (uint8_t)msize;
    out->writes_result = (k != ARM_DEC_STORE);
    out->cycles        = 2;        /* central 1 + the handler's own +1 */
}

int arm_decode_insn(const uint8_t *mem, uint32_t mem_base, uint32_t mem_len,
                    uint32_t pc, arm_decoded_insn_t *out) {
    if (pc < mem_base) return 0;
    uint32_t off = pc - mem_base;
    if (off + 1 >= mem_len) return 0;

    uint16_t hw = (uint16_t)(mem[off] | (mem[off + 1] << 8));
    uint16_t top5 = hw >> 11;

    /* Every 32-bit Thumb-2 encoding is out of scope for v1. */
    if (arm_is_thumb2(hw)) return reject(out, pc, hw);

    switch (top5) {
    /* ---- shift by immediate: LSL/LSR/ASR Rd, Rm, #imm5 ---- */
    case 0: case 1: case 2: {
        /* LSL #0 is the MOV-register form; the interpreter takes a different
         * branch for it (set_nz, no carry update).  Keep v1 honest by only
         * accepting it as an explicit MOV, and rejecting the imm5==0 forms of
         * LSR/ASR (which the interpreter re-reads as a shift of 32). */
        int imm5 = (hw >> 6) & 0x1F;
        base(out, pc, hw, ARM_DEC_SHIFT_IMM);
        out->rd = hw & 7;
        out->rm = (hw >> 3) & 7;
        out->imm = (uint32_t)imm5;
        out->shift = (top5 == 0) ? ARM_SH_LSL
                   : (top5 == 1) ? ARM_SH_LSR : ARM_SH_ASR;
        if (imm5 == 0) {
            if (top5 != 0) return reject(out, pc, hw);  /* LSR/ASR #0 == #32 */
            /* LSL #0 -> plain move, N/Z only, carry untouched. */
            out->klass = ARM_DEC_ALU_REG;
            out->op = ARM_ALU_MOV;
            out->rn = out->rm;
        }
        return 2;
    }

    /* ---- ADD/SUB register, ADD/SUB 3-bit immediate ---- */
    case 3: {
        int sub_op = (hw >> 9) & 3;
        base(out, pc, hw, ARM_DEC_ALU_REG);
        out->rd = hw & 7;
        out->rn = (hw >> 3) & 7;
        switch (sub_op) {
        case 0: out->op = ARM_ALU_ADD; out->rm = (hw >> 6) & 7; break;
        case 1: out->op = ARM_ALU_SUB; out->rm = (hw >> 6) & 7; break;
        case 2: out->op = ARM_ALU_ADD; out->klass = ARM_DEC_ALU_IMM;
                out->imm = (hw >> 6) & 7; break;
        case 3: out->op = ARM_ALU_SUB; out->klass = ARM_DEC_ALU_IMM;
                out->imm = (hw >> 6) & 7; break;
        }
        return 2;
    }

    /* ---- MOV/CMP/ADD/SUB Rd, #imm8 ---- */
    case 4: case 5: case 6: case 7: {
        base(out, pc, hw, ARM_DEC_ALU_IMM);
        out->rd = (hw >> 8) & 7;
        out->rn = out->rd;
        out->imm = hw & 0xFF;
        switch (top5) {
        case 4: out->op = ARM_ALU_MOV; break;
        case 5: out->op = ARM_ALU_CMP; out->writes_result = false; break;
        case 6: out->op = ARM_ALU_ADD; break;
        case 7: out->op = ARM_ALU_SUB; break;
        }
        return 2;
    }

    /* ---- data processing (register), Rd = Rd <op> Rm ---- */
    case 8: {
        if ((hw >> 10) != 0x10) return reject(out, pc, hw); /* hi-reg / BX */
        int opcode = (hw >> 6) & 0xF;
        base(out, pc, hw, ARM_DEC_ALU_REG);
        out->rd = hw & 7;
        out->rn = out->rd;            /* two-operand form */
        out->rm = (hw >> 3) & 7;
        switch (opcode) {
        case 0x0: out->op = ARM_ALU_AND; break;
        case 0x1: out->op = ARM_ALU_EOR; break;
        case 0x8: out->op = ARM_ALU_TST; out->writes_result = false; break;
        case 0x9: out->op = ARM_ALU_RSB; out->rn = out->rm; break; /* Rd = 0-Rm */
        case 0xA: out->op = ARM_ALU_CMP; out->writes_result = false; break;
        case 0xB: out->op = ARM_ALU_CMN; out->writes_result = false; break;
        case 0xC: out->op = ARM_ALU_ORR; break;
        case 0xE: out->op = ARM_ALU_BIC; break;
        case 0xF: out->op = ARM_ALU_MVN; out->rn = out->rm; break;
        /* Excluded on purpose:
         *   0x2-0x4, 0x7  register-controlled shifts (LSL/LSR/ASR/ROR) —
         *                 shift-amount-dependent carry, several edge cases
         *   0x5, 0x6      ADC/SBC — carry-in.  This is the MSP430 ADDC bug.
         *   0xD           MUL — different flag semantics                     */
        default: return reject(out, pc, hw);
        }
        return 2;
    }

    /* ---- conditional branch: terminates a block ----
     * cond 0xE is UDF and 0xF is SVC; the interpreter treats both as NOPs,
     * which is a fidelity gap of its own — not something to reproduce in
     * generated code.  Both are refused. */
    case 26: case 27: {
        int cond = (hw >> 8) & 0xF;
        if (cond >= 0xE) return reject(out, pc, hw);
        int32_t imm8 = (int8_t)(hw & 0xFF);       /* sign-extended */
        base(out, pc, hw, ARM_DEC_B_COND);
        out->writes_result = false;
        out->cond = (uint8_t)cond;
        out->imm  = (uint32_t)(pc + 4 + imm8 * 2);
        return 2;
    }

    /* ---- unconditional branch: terminates a block ---- */
    case 28: {
        int32_t imm11 = hw & 0x7FF;
        if (imm11 & 0x400) imm11 |= (int32_t)0xFFFFF800;   /* sign extend */
        base(out, pc, hw, ARM_DEC_B_UNCOND);
        out->writes_result = false;
        /* arm_step: cpu->reg[ARM_PC] = pc + 4 + imm11 * 2 */
        out->imm = (uint32_t)(pc + 4 + imm11 * 2);
        return 2;
    }

    /* ---- NOP ----
     * Only the bare 0xBF00 encoding.  The rest of the 0xBFxx space is IT (low
     * nibble non-zero) plus WFI/WFE/SEV, all of which have real effects — WFI
     * and WFE stop the CPU, which a compiled block must never do — and the
     * remaining hints are accepted by nobody here rather than reasoned about.
     *
     * It looks like a rounding error and is not: **17.98% of instructions
     * executed by Contiki-NG on cc2538 are this encoding**, and because an
     * unsupported instruction ends a block, it was the single largest thing
     * cutting blocks short on that platform. */
    case 23:
        if (hw != 0xBF00) return reject(out, pc, hw);
        base(out, pc, hw, ARM_DEC_NOP);
        out->writes_result = false;
        return 2;

    /* ---- LDR Rt, [PC, #imm8*4] — literal pool ----
     * The address is a compile-time constant, which is what makes this worth
     * a class of its own: flash is read-only in this model (arm_write32
     * refuses flash writes) and the cache is flushed on reset, so the JIT can
     * resolve the region once and emit an unguarded load. */
    case 9: {
        memop(out, pc, hw, ARM_DEC_LOAD_LIT, (hw >> 8) & 7, 15, 4);
        out->imm = (((pc + 4) & ~3u) + (uint32_t)((hw & 0xFF) << 2));
        return 2;
    }

    /* ---- load/store register offset ---- */
    case 10: case 11: {
        int opcode = (hw >> 9) & 7;
        static const struct { uint8_t klass, msize, sext; } t[8] = {
            { ARM_DEC_STORE, 4, 0 },   /* STR   */
            { ARM_DEC_STORE, 2, 0 },   /* STRH  */
            { ARM_DEC_STORE, 1, 0 },   /* STRB  */
            { ARM_DEC_LOAD,  1, 1 },   /* LDRSB */
            { ARM_DEC_LOAD,  4, 0 },   /* LDR   */
            { ARM_DEC_LOAD,  2, 0 },   /* LDRH  */
            { ARM_DEC_LOAD,  1, 0 },   /* LDRB  */
            { ARM_DEC_LOAD,  2, 1 },   /* LDRSH */
        };
        memop(out, pc, hw, (arm_dec_class_t)t[opcode].klass,
            hw & 7, (hw >> 3) & 7, t[opcode].msize);
        out->sext = t[opcode].sext != 0;
        out->rm   = (uint8_t)((hw >> 6) & 7);
        return 2;
    }

    /* ---- load/store immediate offset, base = Rn ----
     * The scale is the access width, so imm5 addresses imm5 elements. */
    case 12: case 13: case 14: case 15: case 16: case 17: {
        static const struct { uint8_t klass, msize; } t[6] = {
            { ARM_DEC_STORE, 4 }, { ARM_DEC_LOAD, 4 },   /* 12, 13 */
            { ARM_DEC_STORE, 1 }, { ARM_DEC_LOAD, 1 },   /* 14, 15 */
            { ARM_DEC_STORE, 2 }, { ARM_DEC_LOAD, 2 },   /* 16, 17 */
        };
        int k = top5 - 12;
        memop(out, pc, hw, (arm_dec_class_t)t[k].klass,
            hw & 7, (hw >> 3) & 7, t[k].msize);
        out->imm = (uint32_t)(((hw >> 6) & 0x1F) * t[k].msize);
        return 2;
    }

    /* ---- LDR/STR Rt, [SP, #imm8*4] ---- */
    case 18: case 19: {
        memop(out, pc, hw,
            (top5 == 18) ? ARM_DEC_STORE : ARM_DEC_LOAD,
            (hw >> 8) & 7, 13 /* ARM_SP */, 4);
        out->imm = (uint32_t)((hw & 0xFF) << 2);
        return 2;
    }

    default:
        return reject(out, pc, hw);
    }
}

void arm_decode_block(const uint8_t *mem, uint32_t mem_base, uint32_t mem_len,
                      uint32_t pc, arm_basic_block_t *block) {
    memset(block, 0, sizeof(*block));
    block->start_pc      = pc;
    block->end_pc        = pc;
    block->all_supported = true;

    for (int i = 0; i < ARM_MAX_BLOCK_SIZE; i++) {
        arm_decoded_insn_t *di = &block->insns[i];
        int sz = arm_decode_insn(mem, mem_base, mem_len, pc, di);
        if (sz == 0) break;                       /* ran off the image */

        if (di->klass == ARM_DEC_UNSUPPORTED) {
            /* Stop *before* it: the block contains only supported
             * instructions, but is marked so the caller refuses to compile
             * unless it ends at a real terminator. */
            block->all_supported = false;
            break;
        }

        block->length++;
        pc += (uint32_t)sz;
        block->end_pc = pc;

        if (di->klass == ARM_DEC_B_UNCOND ||
            di->klass == ARM_DEC_B_COND) return;   /* terminator */
    }

    /* A block that ran to the cap (or hit an unsupported instruction) has no
     * terminator; the compiler may still run it and fall through to the
     * interpreter at end_pc. */
}
