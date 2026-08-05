/*
 * arm_decode — stateless Thumb instruction decoder + basic-block decoder.
 *
 * WHY THIS EXISTS
 *
 * `arm_step()` fuses decode and execute: it fetches, computes a 5-bit index,
 * computed-gotos into a handler, and the handler extracts its own operand
 * fields inline.  That is fast for pure interpretation — see
 * docs/design/arm-performance-plan.md §4.1, where a decode *cache* measured as
 * a non-optimization because the Thumb-16 dispatch is already `hw >> 11` plus
 * a computed goto — but it means there is no representation of "an
 * instruction" that anything other than the interpreter can consume.
 *
 * A JIT needs exactly that: a block of decoded instructions it can inspect,
 * refuse, and emit native code for.  MSP430 has had one from the start
 * (`msp430_decode.c` -> `decoded_insn_t` -> `msp430_jit_compile`); this is the
 * ARM equivalent, deliberately shaped like it so the two read alike.
 *
 * SCOPE — v1 is intentionally tiny, and that is the safety property
 *
 * This decodes only the subset the JIT is allowed to compile and reports
 * everything else as ARM_DEC_UNSUPPORTED.  The block compiler must refuse any
 * block containing an unsupported instruction, so an instruction not modelled
 * here can never be executed *incorrectly* — only interpreted, as today.
 *
 * Every exclusion that looks over-cautious is there because of one bug: the
 * MSP430 JIT shipped a single correctness defect in its life, and it was
 * `ADDC` — the carry-in consumed the last spare register, leaving nothing to
 * compute the overflow flag with, so V went stale and signed branches flipped.
 * It only appeared once a block went hot, which is why lockstep verification
 * (not the test suite) is what catches this class of bug.
 */
#ifndef ARM_DECODE_H
#define ARM_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Instruction categories for dispatch */
typedef enum {
    ARM_INSN_16BIT,
    ARM_INSN_32BIT,
} arm_insn_width_t;

/* Check if a 16-bit halfword starts a 32-bit Thumb-2 instruction */
static inline int arm_is_thumb2(uint16_t hw) {
    uint16_t top5 = hw >> 11;
    return (top5 == 0x1D || top5 == 0x1E || top5 == 0x1F);
}

/* ---------------------------------------------------------------- */

/* Instruction classes the v1 subset can represent. */
typedef enum {
    ARM_DEC_UNSUPPORTED = 0, /* not in the subset — never compiled           */
    ARM_DEC_ALU_REG,         /* Rd = Rn <op> Rm                              */
    ARM_DEC_ALU_IMM,         /* Rd = Rn <op> #imm                            */
    ARM_DEC_SHIFT_IMM,       /* Rd = Rm <shift> #imm5                        */
    ARM_DEC_B_UNCOND,        /* B <label> — block terminator                 */
} arm_dec_class_t;

/* ALU operations in the v1 subset.  Excludes ADC/SBC (carry-in — the MSP430
 * ADDC lesson) and everything with a memory or PC side effect. */
typedef enum {
    ARM_ALU_AND = 0,
    ARM_ALU_EOR,
    ARM_ALU_ORR,
    ARM_ALU_BIC,
    ARM_ALU_ADD,
    ARM_ALU_SUB,
    ARM_ALU_RSB,     /* Rd = 0 - Rm (NEGS)                                   */
    ARM_ALU_MOV,
    ARM_ALU_MVN,     /* Rd = ~Rm                                             */
    ARM_ALU_CMP,     /* flags only                                           */
    ARM_ALU_CMN,     /* flags only                                           */
    ARM_ALU_TST,     /* flags only                                           */
} arm_alu_op_t;

typedef enum {
    ARM_SH_LSL = 0,
    ARM_SH_LSR,
    ARM_SH_ASR,
} arm_shift_t;

typedef struct arm_decoded_insn {
    uint32_t        pc;         /* address of this instruction               */
    uint16_t        raw;        /* the halfword, for diagnostics             */
    uint8_t         size;       /* 2 — the v1 subset has no 32-bit forms     */
    arm_dec_class_t klass;
    arm_alu_op_t    op;
    arm_shift_t     shift;      /* SHIFT_IMM only                            */

    uint8_t  rd, rn, rm;        /* register operands (low regs in v1)        */
    uint32_t imm;               /* immediate, shift amount, or branch target */

    bool     writes_result;     /* false for CMP/CMN/TST                     */
    uint8_t  cycles;            /* cycles the interpreter charges            */
} arm_decoded_insn_t;

/* A block is a straight-line run ending at a terminator (or at the cap).
 * 32 matches MSP430's MAX_BLOCK_SIZE so the two tune alike. */
#define ARM_MAX_BLOCK_SIZE 32

typedef struct arm_basic_block {
    arm_decoded_insn_t insns[ARM_MAX_BLOCK_SIZE];
    int      length;
    uint32_t start_pc;
    uint32_t end_pc;            /* first address past the block              */
    bool     all_supported;     /* false -> the JIT must refuse this block   */
} arm_basic_block_t;

/*
 * Decode one Thumb-16 instruction at `pc`, reading from the flash image `mem`
 * which spans [mem_base, mem_base + mem_len).
 *
 * Always writes `out`.  Sets klass = ARM_DEC_UNSUPPORTED for anything outside
 * the v1 subset, including every 32-bit Thumb-2 encoding.  Returns the size in
 * bytes (2), or 0 if `pc` (or `pc+1`) is outside the image.
 */
int arm_decode_insn(const uint8_t *mem, uint32_t mem_base, uint32_t mem_len,
                    uint32_t pc, arm_decoded_insn_t *out);

/*
 * Decode a straight-line block starting at `pc`.  Stops after a terminator,
 * on the first unsupported instruction, or at ARM_MAX_BLOCK_SIZE.
 * `all_supported` says whether the whole block is compilable; the caller must
 * not compile a block where it is false.
 */
void arm_decode_block(const uint8_t *mem, uint32_t mem_base, uint32_t mem_len,
                      uint32_t pc, arm_basic_block_t *block);

/*
 * Reference semantics for the subset — the contract the JIT's generated code
 * must reproduce exactly (registers, APSR, cycles).  Implemented in
 * arm_cpu.c so it shares the interpreter's own flag helpers rather than
 * re-deriving them; `arm-decode` differential-tests it against arm_step().
 *
 * The caller must already have advanced PC and charged the cycle, exactly as
 * arm_step does centrally before dispatch.  Returns 1 if executed, 0 if the
 * instruction is not in the subset.
 */
struct arm_cpu;
int arm_execute_decoded(struct arm_cpu *cpu, const arm_decoded_insn_t *di);

#ifdef __cplusplus
}
#endif

#endif /* ARM_DECODE_H */
