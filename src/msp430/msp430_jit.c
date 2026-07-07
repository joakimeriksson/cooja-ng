/*
 * MSP430 JIT compiler using GNU Lightning
 *
 * Compiles hot basic blocks into native machine code.
 * Handles: reg-to-reg two-op, CG/imm-to-reg two-op, jumps.
 * Falls back to C interpreter for memory-accessing instructions.
 */
#ifdef HAVE_LIGHTNING

#include "msp430_jit.h"
#include "msp430_cpu.h"
#include "msp430_decode.h"
#include <lightning.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

/*
 * Register allocation:
 *   JIT_V0 = msp430_cpu_t *cpu (callee-saved, persistent)
 *   JIT_V1 = cpu->reg base pointer (callee-saved)
 *   JIT_V2 = cpu->cycles (callee-saved, loaded at entry, stored at exit)
 *   JIT_R0, JIT_R1, JIT_R2 = temporaries
 *
 * On entry: cpu pointer is passed as first argument
 * On exit:  return number of instructions executed
 */

/* Offsets into msp430_cpu_t for fields we access */
#define OFF_REG        offsetof(msp430_cpu_t, reg)
#define OFF_CYCLES     offsetof(msp430_cpu_t, cycles)
#define OFF_SR         (MSP430_SR * sizeof(uint32_t))
#define OFF_PC         (MSP430_PC * sizeof(uint32_t))

/* Offsets for interrupt/event checking inside JIT blocks */
#define OFF_INTERRUPT_MAX       offsetof(msp430_cpu_t, interrupt_max)
#define OFF_INTERRUPTS_ENABLED  offsetof(msp430_cpu_t, interrupts_enabled)
#define OFF_NEXT_EVENT_CYCLE    offsetof(msp430_cpu_t, next_event_cycle)

/* Check for interrupts/events every N instructions inside JIT blocks */
#define JIT_INTERRUPT_CHECK_INTERVAL 10

/* Helper: emit code to load reg[n] into a JIT temp register */
#define EMIT_LOAD_REG(tmp, regnum) \
    jit_ldxi_i(tmp, JIT_V1, (regnum) * sizeof(uint32_t))

/* Helper: emit code to store JIT temp register into reg[n] */
#define EMIT_STORE_REG(tmp, regnum) \
    jit_stxi_i((regnum) * sizeof(uint32_t), JIT_V1, tmp)

/*
 * Check if an instruction can be inlined by the JIT.
 * Returns true if the instruction can be compiled to native code.
 */
static int can_inline(const decoded_insn_t *di) {
    if (di->itype == ITYPE_JUMP) return 1;

    if (di->itype == ITYPE_TWO_OP && di->ext_word == 0) {
        /* Can inline: reg/CG/imm source with register destination */
        if (di->dst_kind != OPKIND_REG) return 0;
        if (di->dst_reg == MSP430_SR) return 0; /* SR writes have side effects */
        if (di->dst_reg == MSP430_PC) return 0; /* PC writes need special handling */

        /* Reject operations we can't inline correctly.  ADDC joins SUBC/DADD:
         * its carry-in occupies the one spare temp (JIT_V2), leaving no
         * register to compute the overflow (V) flag the way OP_ADD does, so
         * an inlined ADDC left V stale — a warm-block-only divergence from the
         * interpreter that flipped signed branches (JGE/JL) after an ADDC.
         * The interpreter computes ADDC's V correctly; let it. */
        if (di->operation == OP_SUBC || di->operation == OP_DADD ||
            di->operation == OP_ADDC) return 0;

        switch (di->src_kind) {
        case OPKIND_REG:
        case OPKIND_CG:
        case OPKIND_IMMEDIATE:
            return 1;
        default:
            return 0;
        }
    }

    return 0;
}

/*
 * Emit the ALU operation + flags for a two-operand instruction.
 *
 * On entry:
 *   JIT_R0 = src value (already masked)
 *   JIT_R1 = dst value (already masked, or 0 for MOV)
 *   JIT_R2 = current SR
 *   di->operation = opcode
 *   di->byte_mode = 0 or 1
 *
 * On exit:
 *   JIT_R1 = result value (masked)
 *   JIT_R2 = updated SR
 *   Returns: 1 if result should be written back, 0 if not (CMP/BIT)
 */
static int emit_alu_and_flags(jit_state_t *_jit, const decoded_insn_t *di) {
    uint32_t mask = di->byte_mode ? 0xff : 0xffff;
    uint32_t msb  = di->byte_mode ? 0x80 : 0x8000;
    int write_back = 1;

    switch (di->operation) {
    case OP_MOV:
        /* dst = src, no flags update */
        jit_movr(JIT_R1, JIT_R0);
        return 1;

    case OP_ADD: {
        /* Clear V, C in SR */
        jit_andi(JIT_R2, JIT_R2, ~(SR_V | SR_C));
        /* V flag: save (src ^ dst) before ADD */
        jit_xorr(JIT_V2, JIT_R0, JIT_R1);  /* V2 = src ^ dst (pre-add) */
        /* dst = dst + src */
        jit_addr(JIT_R1, JIT_R1, JIT_R0);
        /* C flag: if result > mask */
        jit_node_t *no_carry = jit_blei_u(JIT_R1, (jit_word_t)mask);
        jit_ori(JIT_R2, JIT_R2, SR_C);
        jit_patch(no_carry);
        /* V flag: (src ^ dst_pre) & msb == 0 && (src ^ result) & msb != 0 */
        jit_node_t *no_v_add = jit_bmsi(JIT_V2, (jit_word_t)msb);
        jit_xorr(JIT_V2, JIT_R0, JIT_R1);  /* V2 = src ^ result */
        jit_node_t *no_v_add2 = jit_bmci(JIT_V2, (jit_word_t)msb);
        jit_ori(JIT_R2, JIT_R2, SR_V);
        jit_patch(no_v_add2);
        jit_patch(no_v_add);
        /* Mask result */
        jit_andi(JIT_R1, JIT_R1, mask);
        /* Restore V2 for cycle accumulator */
        jit_movi(JIT_V2, 0);
        break;
    }

    case OP_ADDC: {
        /* ADDC: dst = dst + src + carry.
         * Extract carry BEFORE clearing SR flags. */
        jit_andi(JIT_V2, JIT_R2, SR_C);   /* V2 = old carry (0 or 1) */
        jit_andi(JIT_R2, JIT_R2, ~(SR_V | SR_C));
        /* dst = dst + src + carry */
        jit_addr(JIT_R1, JIT_R1, JIT_R0);
        jit_addr(JIT_R1, JIT_R1, JIT_V2);
        /* C flag: if result > mask */
        jit_node_t *no_carry_addc = jit_blei_u(JIT_R1, (jit_word_t)mask);
        jit_ori(JIT_R2, JIT_R2, SR_C);
        jit_patch(no_carry_addc);
        /* V flag: skip for ADDC (no temp register available; rarely tested) */
        /* Mask result */
        jit_andi(JIT_R1, JIT_R1, mask);
        jit_movi(JIT_V2, 0);
        break;
    }

    case OP_SUB:
    case OP_CMP: {
        /* SUB/CMP: src = (src ^ 0xFFFF) & 0xFFFF, then dst = dst + src + 1
         * Match MSPSim: 16-bit complement even for byte mode (affects carry) */
        jit_xori(JIT_R0, JIT_R0, 0xFFFF);
        jit_andi(JIT_R0, JIT_R0, 0xFFFF);
        jit_andi(JIT_R2, JIT_R2, ~(SR_V | SR_C));
        /* V flag: save (src_inv ^ dst) before add */
        jit_xorr(JIT_V2, JIT_R0, JIT_R1);  /* V2 = src_inv ^ dst */
        jit_addr(JIT_R1, JIT_R1, JIT_R0);
        jit_addi(JIT_R1, JIT_R1, 1);
        /* C flag */
        jit_node_t *no_carry_sub = jit_blei_u(JIT_R1, (jit_word_t)mask);
        jit_ori(JIT_R2, JIT_R2, SR_C);
        jit_patch(no_carry_sub);
        /* V flag */
        jit_node_t *no_v_sub = jit_bmsi(JIT_V2, (jit_word_t)msb);
        jit_xorr(JIT_V2, JIT_R0, JIT_R1);
        jit_node_t *no_v_sub2 = jit_bmci(JIT_V2, (jit_word_t)msb);
        jit_ori(JIT_R2, JIT_R2, SR_V);
        jit_patch(no_v_sub2);
        jit_patch(no_v_sub);
        jit_andi(JIT_R1, JIT_R1, mask);
        if (di->operation == OP_CMP) write_back = 0;
        /* Restore V2 */
        jit_movi(JIT_V2, 0);
        break;
    }

    case OP_BIC:
        /* dst = dst & ~src */
        jit_comr(JIT_R0, JIT_R0);
        jit_andr(JIT_R1, JIT_R1, JIT_R0);
        jit_andi(JIT_R1, JIT_R1, mask);
        return 1; /* No flags update */

    case OP_BIS:
        /* dst = src | dst */
        jit_orr(JIT_R1, JIT_R1, JIT_R0);
        jit_andi(JIT_R1, JIT_R1, mask);
        return 1; /* No flags update */

    case OP_AND: {
        jit_andi(JIT_R2, JIT_R2, ~(SR_C | SR_V));
        jit_andr(JIT_R1, JIT_R1, JIT_R0);
        jit_andi(JIT_R1, JIT_R1, mask);
        /* C = (result != 0) */
        jit_node_t *and_zero = jit_beqi(JIT_R1, 0);
        jit_ori(JIT_R2, JIT_R2, SR_C);
        jit_patch(and_zero);
        break;
    }

    case OP_XOR: {
        jit_andi(JIT_R2, JIT_R2, ~(SR_C | SR_V));
        /* V = both src and dst have MSB set */
        /* Test src & msb */
        jit_node_t *xor_no_v1 = jit_bmci(JIT_R0, (jit_word_t)msb);
        jit_node_t *xor_no_v2 = jit_bmci(JIT_R1, (jit_word_t)msb);
        jit_ori(JIT_R2, JIT_R2, SR_V);
        jit_patch(xor_no_v2);
        jit_patch(xor_no_v1);
        jit_xorr(JIT_R1, JIT_R1, JIT_R0);
        jit_andi(JIT_R1, JIT_R1, mask);
        /* C = (result != 0) */
        jit_node_t *xor_zero = jit_beqi(JIT_R1, 0);
        jit_ori(JIT_R2, JIT_R2, SR_C);
        jit_patch(xor_zero);
        break;
    }

    case OP_BIT: {
        jit_andi(JIT_R2, JIT_R2, ~(SR_C | SR_V));
        jit_andr(JIT_R1, JIT_R1, JIT_R0);
        jit_andi(JIT_R1, JIT_R1, mask);
        jit_node_t *bit_zero = jit_beqi(JIT_R1, 0);
        jit_ori(JIT_R2, JIT_R2, SR_C);
        jit_patch(bit_zero);
        write_back = 0;
        break;
    }

    case OP_SUBC:
    case OP_DADD:
        /* These are complex (carry-in dependent / BCD), fall back */
        return -1;

    default:
        return -1;
    }

    /* Update Z and N flags */
    jit_andi(JIT_R2, JIT_R2, ~(SR_Z | SR_N));
    /* Z flag */
    jit_node_t *not_zero = jit_bnei(JIT_R1, 0);
    jit_ori(JIT_R2, JIT_R2, SR_Z);
    jit_patch(not_zero);
    /* N flag */
    jit_node_t *not_neg = jit_bmci(JIT_R1, (jit_word_t)msb);
    jit_ori(JIT_R2, JIT_R2, SR_N);
    jit_patch(not_neg);

    /* Store updated SR */
    EMIT_STORE_REG(JIT_R2, MSP430_SR);

    return write_back;
}

/* Forward declaration for the C fallback function */
extern int execute_decoded_for_jit(msp430_cpu_t *cpu, const decoded_insn_t *di,
                                    uint32_t next_pc);

compiled_block_t *msp430_jit_compile(basic_block_t *block, msp430_cpu_t *cpu) {
    /* Only compile blocks where ALL instructions can be inlined.
     * Mixing inlined and fallback instructions is unsafe because
     * the fallback path can trigger interrupts that change control flow. */
    for (int i = 0; i < block->length; i++) {
        if (!can_inline(&block->insns[i])) return NULL;
    }

    jit_state_t *_jit = jit_new_state();
    if (!_jit) return NULL;

    jit_prolog();

    /* Get the cpu argument */
    jit_node_t *arg = jit_arg();
    jit_getarg(JIT_V0, arg);

    /* JIT_V1 = &cpu->reg[0] */
    jit_addi(JIT_V1, JIT_V0, OFF_REG);

    /* JIT_V2 = cpu->cycles (as 32-bit low part for cycle counting) */
    /* We'll accumulate cycles in JIT_V2 and store at the end */
    jit_movi(JIT_V2, 0);

    /* Track instruction labels for intra-block jumps */
    jit_node_t *insn_labels[MAX_BLOCK_SIZE + 1];

    /* Emit code for each instruction */
    int inst_count = block->length;

    for (int i = 0; i < block->length; i++) {
        const decoded_insn_t *di = &block->insns[i];
        uint32_t next_pc = (i + 1 < block->length) ?
                           block->pc_addrs[i + 1] : block->end_pc;

        insn_labels[i] = jit_label();

        /* Periodic interrupt/event check every N instructions.
         * Flushes cycles, checks for pending events or interrupts,
         * and bails out returning partial instruction count if needed.
         * Only emitted when jit_inblock_checks is enabled. */
        if (cpu->jit_inblock_checks &&
            i > 0 && (i % JIT_INTERRUPT_CHECK_INTERVAL) == 0) {
            /* Flush accumulated cycles */
            jit_ldxi_l(JIT_R0, JIT_V0, OFF_CYCLES);
            jit_addr(JIT_R0, JIT_R0, JIT_V2);
            jit_stxi_l(OFF_CYCLES, JIT_V0, JIT_R0);
            jit_movi(JIT_V2, 0);

            /* Check: cpu->cycles >= cpu->next_event_cycle? */
            /* JIT_R0 already has updated cpu->cycles */
            jit_ldxi_l(JIT_R1, JIT_V0, OFF_NEXT_EVENT_CYCLE);
            jit_node_t *no_event = jit_bltr(JIT_R0, JIT_R1);
            jit_reti(i);  /* Return instructions executed so far */
            jit_patch(no_event);

            /* Check: interrupt pending (interrupt_max >= 0 && interrupts_enabled)? */
            jit_ldxi_i(JIT_R0, JIT_V0, OFF_INTERRUPT_MAX);
            jit_node_t *no_int = jit_blti(JIT_R0, 0);
            jit_ldxi_uc(JIT_R0, JIT_V0, OFF_INTERRUPTS_ENABLED);
            jit_node_t *no_ie = jit_beqi(JIT_R0, 0);
            jit_reti(i);  /* Bail out for interrupt servicing */
            jit_patch(no_ie);
            jit_patch(no_int);
        }

        if (!can_inline(di)) {
            /* Fall back: store accumulated cycles, call C, reload */
            /* Store cycles: cpu->cycles += JIT_V2 */
            jit_ldxi_l(JIT_R0, JIT_V0, OFF_CYCLES);
            jit_addr(JIT_R0, JIT_R0, JIT_V2);
            jit_stxi_l(OFF_CYCLES, JIT_V0, JIT_R0);
            jit_movi(JIT_V2, 0);

            /* Call execute_decoded_for_jit(cpu, &block->insns[i], next_pc) */
            jit_prepare();
            jit_pushargr(JIT_V0);
            jit_pushargi((jit_word_t)di);
            jit_pushargi((jit_word_t)next_pc);
            jit_finishi((jit_pointer_t)execute_decoded_for_jit);

            /* Reload V1 (reg pointer) — shouldn't change but be safe */
            jit_addi(JIT_V1, JIT_V0, OFF_REG);
            continue;
        }

        if (di->itype == ITYPE_JUMP) {
            /* Add 2 cycles for jump */
            jit_addi(JIT_V2, JIT_V2, 2);

            /* Load SR */
            EMIT_LOAD_REG(JIT_R0, MSP430_SR);

            /* Compute target PC */
            uint32_t target_pc = (next_pc + di->jmp_offset) &
                                 (cpu->is_msp430x ? 0xfffff : 0xffff);
            int cond = di->jmp_condition;

            if (cond == JMP_JMP) {
                /* Unconditional jump */
                jit_movi(JIT_R0, target_pc);
                EMIT_STORE_REG(JIT_R0, MSP430_PC);
                continue;
            }

            /* Conditional: evaluate and branch to "taken" label */
            jit_node_t *taken = NULL;

            switch (cond) {
            case JMP_JNE:
                taken = jit_bmci(JIT_R0, SR_Z);
                break;
            case JMP_JEQ:
                taken = jit_bmsi(JIT_R0, SR_Z);
                break;
            case JMP_JNC:
                taken = jit_bmci(JIT_R0, SR_C);
                break;
            case JMP_JC:
                taken = jit_bmsi(JIT_R0, SR_C);
                break;
            case JMP_JN:
                taken = jit_bmsi(JIT_R0, SR_N);
                break;
            case JMP_JGE:
            case JMP_JL: {
                /* JGE: jump if (N!=0)==(V!=0), i.e. N XOR V == 0
                 * JL:  jump if (N!=0)!=(V!=0), i.e. N XOR V != 0
                 * Extract N (bit 2) and V (bit 8), normalize to 0/1,
                 * XOR them, and branch on result. */
                jit_andi(JIT_R1, JIT_R0, SR_N);  /* R1 = N bit (0 or SR_N) */
                jit_rshi_u(JIT_R1, JIT_R1, 2);   /* R1 = 0 or 1 */
                jit_andi(JIT_R2, JIT_R0, SR_V);  /* R2 = V bit (0 or SR_V) */
                jit_rshi_u(JIT_R2, JIT_R2, 8);   /* R2 = 0 or 1 */
                jit_xorr(JIT_R1, JIT_R1, JIT_R2);/* R1 = N XOR V (0 or 1) */
                if (cond == JMP_JGE) {
                    taken = jit_beqi(JIT_R1, 0);  /* JGE: jump if N==V */
                } else {
                    taken = jit_bnei(JIT_R1, 0);  /* JL: jump if N!=V */
                }
                break;
            }
            }

            /* Not taken path: store next_pc */
            jit_movi(JIT_R0, next_pc);
            EMIT_STORE_REG(JIT_R0, MSP430_PC);
            jit_node_t *skip_taken = jit_jmpi();

            /* Taken path: store target_pc */
            jit_patch(taken);
            jit_movi(JIT_R0, target_pc);
            EMIT_STORE_REG(JIT_R0, MSP430_PC);

            jit_patch(skip_taken);
            continue;
        }

        /* Two-operand instruction (reg/CG/imm src, reg dst) */
        if (di->itype == ITYPE_TWO_OP) {
            uint32_t mask = di->byte_mode ? 0xff : 0xffff;

            /* Determine cycles (C-level only, no JIT emission) */
            int cycles;
            switch (di->src_kind) {
            case OPKIND_REG:       cycles = 1; break;
            case OPKIND_CG:        cycles = 1; break;
            case OPKIND_IMMEDIATE: cycles = 2; break;
            default:               cycles = 1; break;
            }

            /* Add cycles to accumulator */
            jit_addi(JIT_V2, JIT_V2, cycles);

            /* For ops that use JIT_V2 as temp (ADD/ADDC/SUB/CMP), flush
             * cycle accumulator to cpu->cycles first */
            if (di->operation == OP_ADD || di->operation == OP_ADDC ||
                di->operation == OP_SUB || di->operation == OP_CMP) {
                jit_ldxi_l(JIT_R0, JIT_V0, OFF_CYCLES);
                jit_addr(JIT_R0, JIT_R0, JIT_V2);
                jit_stxi_l(OFF_CYCLES, JIT_V0, JIT_R0);
                jit_movi(JIT_V2, 0);
            }

            /* Load source into JIT_R0 */
            switch (di->src_kind) {
            case OPKIND_REG:
                EMIT_LOAD_REG(JIT_R0, di->src_reg);
                jit_andi(JIT_R0, JIT_R0, mask);
                break;
            case OPKIND_CG: {
                int32_t val = di->src_value;
                if (di->byte_mode && di->src_reg == MSP430_CG2 && val == (int32_t)0xFFFF)
                    val = 0xff;
                jit_movi(JIT_R0, val);
                break;
            }
            case OPKIND_IMMEDIATE:
                jit_movi(JIT_R0, di->src_value & mask);
                break;
            default:
                /* Should not happen — can_inline checked */
                continue;
            }

            /* Load dst into JIT_R1 (skip for MOV) */
            if (di->operation != OP_MOV) {
                EMIT_LOAD_REG(JIT_R1, di->dst_reg);
                jit_andi(JIT_R1, JIT_R1, mask);
            } else {
                jit_movi(JIT_R1, 0);
            }

            /* Load SR into JIT_R2 */
            EMIT_LOAD_REG(JIT_R2, MSP430_SR);

            /* Emit ALU + flags */
            int wb = emit_alu_and_flags(_jit, di);
            if (wb < 0) {
                /* Couldn't inline — shouldn't happen, but fall back */
                /* This case means can_inline returned true for something
                   emit_alu_and_flags can't handle (ADDC/SUBC/DADD) */
                /* Store cycles, call C fallback */
                jit_ldxi_l(JIT_R0, JIT_V0, OFF_CYCLES);
                jit_addr(JIT_R0, JIT_R0, JIT_V2);
                jit_stxi_l(OFF_CYCLES, JIT_V0, JIT_R0);
                jit_movi(JIT_V2, 0);
                jit_prepare();
                jit_pushargr(JIT_V0);
                jit_pushargi((jit_word_t)di);
                jit_pushargi((jit_word_t)next_pc);
                jit_finishi((jit_pointer_t)execute_decoded_for_jit);
                jit_addi(JIT_V1, JIT_V0, OFF_REG);
                continue;
            }

            /* Write back result */
            if (wb) {
                EMIT_STORE_REG(JIT_R1, di->dst_reg);
            }

            /* Store next PC */
            jit_movi(JIT_R0, next_pc);
            EMIT_STORE_REG(JIT_R0, MSP430_PC);
        }
    }

    /* Epilogue: store accumulated cycles and return instruction count */
    insn_labels[block->length] = jit_label();
    jit_ldxi_l(JIT_R0, JIT_V0, OFF_CYCLES);
    jit_addr(JIT_R0, JIT_R0, JIT_V2);
    jit_stxi_l(OFF_CYCLES, JIT_V0, JIT_R0);
    jit_reti(inst_count);

    jit_epilog();

    compiled_fn fn = (compiled_fn)jit_emit();
    if (!fn) {
        jit_destroy_state();
        return NULL;
    }

    /* Keep _jit alive (it owns the code memory) */
    /* jit_clear_state() frees IR but keeps code */
    jit_clear_state();

    compiled_block_t *cb = (compiled_block_t *)malloc(sizeof(compiled_block_t));
    cb->fn = fn;
    cb->length = inst_count;
    cb->jit_state = _jit;

    return cb;
}

void msp430_jit_free(compiled_block_t *cb) {
    if (!cb) return;
    if (cb->jit_state) {
        jit_state_t *_jit = cb->jit_state;
        jit_destroy_state();
    }
    free(cb);
}

void msp430_jit_init(void) {
    init_jit(NULL);
}

void msp430_jit_finish(void) {
    finish_jit();
}

#endif /* HAVE_LIGHTNING */
