/*
 * MSP430 instruction decoder — decoded instruction and basic block types
 *
 * Stateless decoder: memory -> decoded_insn_t -> basic_block_t
 * Used by the decode cache and JIT compiler.
 */
#ifndef MSP430_DECODE_H
#define MSP430_DECODE_H

#include <stdint.h>

/* Instruction types */
#define ITYPE_TWO_OP      0   /* Two-operand: MOV, ADD, SUB, etc. */
#define ITYPE_SINGLE_OP   1   /* Single-operand: RRC, PUSH, CALL, etc. */
#define ITYPE_JUMP        2   /* Conditional/unconditional jumps */
#define ITYPE_MSP430X     3   /* MSP430X native instructions (MOVA, etc.) */

/* Source/dst operand kinds (resolved from As/Ad + register) */
#define OPKIND_REG         0   /* Register direct */
#define OPKIND_CG          1   /* Constant generator value */
#define OPKIND_INDEXED     2   /* Indexed: offset(Rn) */
#define OPKIND_ABSOLUTE    3   /* Absolute: &addr */
#define OPKIND_INDIRECT    4   /* Indirect: @Rn */
#define OPKIND_AUTOINC     5   /* Auto-increment: @Rn+ */
#define OPKIND_IMMEDIATE   6   /* Immediate: #value */

typedef struct decoded_insn {
    uint16_t instruction;      /* Raw instruction word */
    uint8_t  itype;            /* ITYPE_* */
    uint8_t  operation;        /* Opcode: OP_MOV..OP_AND for two-op,
                                  single-op code for single-op */
    uint8_t  byte_mode;        /* 0=word, 1=byte */

    /* Source operand */
    uint8_t  src_kind;         /* OPKIND_* */
    uint8_t  src_reg;          /* Source register (0-15) */
    int32_t  src_value;        /* CG value, index offset, or immediate */

    /* Destination operand */
    uint8_t  dst_kind;         /* OPKIND_* */
    uint8_t  dst_reg;          /* Destination register (0-15) */
    int32_t  dst_value;        /* Index offset for indexed dst */

    /* Jump fields */
    uint8_t  jmp_condition;    /* JMP_JNE..JMP_JMP */
    int16_t  jmp_offset;       /* Signed PC-relative offset (bytes) */

    /* Size: how many bytes this instruction occupies */
    uint8_t  size;             /* 2, 4, or 6 bytes */

    /* MSP430X extension */
    uint16_t ext_word;         /* Extension word (0 if none) */
} decoded_insn_t;

#define MAX_BLOCK_SIZE 32

typedef struct basic_block {
    decoded_insn_t insns[MAX_BLOCK_SIZE];
    uint32_t       pc_addrs[MAX_BLOCK_SIZE];  /* PC of each instruction */
    int            length;                     /* Number of instructions */
    uint32_t       start_pc;
    uint32_t       end_pc;                     /* PC after last instruction */
} basic_block_t;

/*
 * Decode one instruction at `pc`.
 * Returns instruction byte count (2, 4, or 6).
 * Fills `out` with decoded fields.
 */
int msp430_decode(const uint8_t *mem, uint32_t pc, uint32_t max_mem,
                  decoded_insn_t *out);

/*
 * Decode a basic block starting at `pc`.
 * Terminates at: jump, CALL, RETI, write to PC/SR as dst, or MAX_BLOCK_SIZE.
 * Returns block length (number of instructions).
 */
int msp430_decode_block(const uint8_t *mem, uint32_t pc, uint32_t max_mem,
                        basic_block_t *block);

#endif /* MSP430_DECODE_H */
