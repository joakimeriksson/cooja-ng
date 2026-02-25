/*
 * MSP430 instruction decoder — stateless decoder
 *
 * Decodes raw memory bytes into decoded_insn_t structs.
 */
#include "msp430_decode.h"
#include "msp430_cpu.h"
#include <string.h>

static inline uint16_t read_w(const uint8_t *mem, uint32_t addr) {
    return (uint16_t)(mem[addr] | (mem[addr + 1] << 8));
}

static inline int16_t sign_ext_16(int val) {
    if (val & 0x8000) return (int16_t)(val | 0xffff0000);
    return (int16_t)val;
}

/* CG values: CG1 (R2): As=2->4, As=3->8.  CG2 (R3): As=0->0, As=1->1, As=2->2, As=3->0xFFFF */
static const int32_t cg1_vals[4] = { 0, 0, 4, 8 };
static const int32_t cg2_vals[4] = { 0, 1, 2, (int32_t)0xFFFF };

int msp430_decode(const uint8_t *mem, uint32_t pc, uint32_t max_mem,
                  decoded_insn_t *out) {
    memset(out, 0, sizeof(*out));

    if (pc + 1 >= max_mem) {
        out->size = 2;
        return 2;
    }

    uint16_t instr = read_w(mem, pc);
    out->instruction = instr;
    int size = 2;

    int op = (instr >> 12) & 0xf;

    /* Check for extension word prefix */
    if ((instr & 0xf800) == 0x1800) {
        out->ext_word = instr;
        /* Fetch the actual instruction */
        uint16_t real_instr = read_w(mem, pc + 2);
        out->instruction = real_instr;
        size = 4; /* At least 4 bytes (ext word + instruction) */
        int real_op = (real_instr >> 12) & 0xf;

        if (real_op >= 4) {
            /* Extended two-operand instruction */
            out->itype = ITYPE_TWO_OP;
            out->operation = real_op;
            out->byte_mode = (real_instr & 0x0040) ? 1 : 0;

            int src_register = (real_instr >> 8) & 0xf;
            int dst_register = real_instr & 0xf;
            int ad = (real_instr >> 7) & 0x1;
            int as = (real_instr >> 4) & 0x3;

            /* Resolve source */
            if (src_register == MSP430_CG2) {
                out->src_kind = OPKIND_CG;
                out->src_reg = src_register;
                out->src_value = cg2_vals[as];
            } else if (src_register == MSP430_CG1 && as >= 2) {
                out->src_kind = OPKIND_CG;
                out->src_reg = src_register;
                out->src_value = cg1_vals[as];
            } else {
                out->src_reg = src_register;
                switch (as) {
                case AM_REG:
                    out->src_kind = OPKIND_REG;
                    break;
                case AM_INDEX:
                    if (src_register == MSP430_SR) {
                        out->src_kind = OPKIND_ABSOLUTE;
                    } else {
                        out->src_kind = OPKIND_INDEXED;
                    }
                    out->src_value = sign_ext_16(read_w(mem, pc + size));
                    size += 2;
                    break;
                case AM_IND_REG:
                    out->src_kind = OPKIND_INDIRECT;
                    break;
                case AM_IND_AUTOINC:
                    if (src_register == MSP430_PC) {
                        out->src_kind = OPKIND_IMMEDIATE;
                        out->src_value = read_w(mem, pc + size);
                        size += 2;
                    } else {
                        out->src_kind = OPKIND_AUTOINC;
                    }
                    break;
                }
            }

            /* Resolve destination */
            out->dst_reg = dst_register;
            if (ad == 0) {
                out->dst_kind = OPKIND_REG;
            } else {
                if (dst_register == MSP430_SR) {
                    out->dst_kind = OPKIND_ABSOLUTE;
                } else {
                    out->dst_kind = OPKIND_INDEXED;
                }
                out->dst_value = sign_ext_16(read_w(mem, pc + size));
                size += 2;
            }
        } else {
            /* Extended single-op (rare but possible) */
            out->itype = ITYPE_MSP430X;
            out->operation = real_op;
        }

        out->size = size;
        return size;
    }

    /* MSP430X native instructions (opcode 0, not extension word) */
    if (op == 0) {
        out->itype = ITYPE_MSP430X;
        out->operation = 0;

        /* CALLA, PUSHM/POPM, RRXX, MOVA/CMPA/ADDA/SUBA all live here */
        /* For block decoding purposes, just record the size */
        int msp430x_op = instr & 0x00f0;

        /* CALLA variants */
        if ((instr & 0xff00) == 0x1300 && (instr & 0x00f0) >= 0x0040) {
            int calla_op = instr & 0xfff0;
            if (calla_op == CALLA_INDEX || calla_op == CALLA_IMM ||
                calla_op == CALLA_ABS || calla_op == CALLA_EDE) {
                size = 4;
            }
            out->size = size;
            return size;
        }

        /* PUSHM/POPM */
        if ((instr & 0xfc00) >= 0x1400 && (instr & 0xfc00) <= 0x1700) {
            out->size = 2;
            return 2;
        }

        /* RRXX */
        if ((instr & 0xf000) == 0x0000 && (instr & 0x0c00) != 0) {
            out->size = 2;
            return 2;
        }

        /* MOVA/CMPA/ADDA/SUBA */
        switch (msp430x_op) {
        case MOVA_ABS2REG: case MOVA_INDX2REG:
        case MOVA_REG2ABS: case MOVA_REG2INDX:
        case MOVA_IMM2REG: case CMPA_IMM:
        case ADDA_IMM: case SUBA_IMM:
            size = 4;
            break;
        default:
            size = 2;
            break;
        }

        out->size = size;
        return size;
    }

    /* Single-operand instructions (opcode 1) */
    if (op == 1) {
        out->itype = ITYPE_SINGLE_OP;
        int single_op = instr & 0x1380;
        out->operation = single_op >> 7; /* compact representation */
        out->byte_mode = (instr & 0x0040) ? 1 : 0;

        int dst_register = instr & 0xf;
        int ad = (instr >> 4) & 0x3;

        out->dst_reg = dst_register;

        /* RETI is special — no operand */
        if (single_op == OP_RETI) {
            out->operation = single_op >> 7;
            out->dst_kind = OPKIND_REG;
            out->size = 2;
            return 2;
        }

        /* Resolve operand addressing mode */
        if ((dst_register == MSP430_CG1 && ad > AM_INDEX) ||
            dst_register == MSP430_CG2) {
            out->dst_kind = OPKIND_CG;
            out->dst_value = (dst_register == MSP430_CG2) ?
                             cg2_vals[ad] : cg1_vals[ad];
        } else {
            switch (ad) {
            case AM_REG:
                out->dst_kind = OPKIND_REG;
                break;
            case AM_INDEX:
                if (dst_register == MSP430_SR) {
                    out->dst_kind = OPKIND_ABSOLUTE;
                } else {
                    out->dst_kind = OPKIND_INDEXED;
                }
                out->dst_value = sign_ext_16(read_w(mem, pc + size));
                size += 2;
                break;
            case AM_IND_REG:
                out->dst_kind = OPKIND_INDIRECT;
                break;
            case AM_IND_AUTOINC:
                if (dst_register == MSP430_PC) {
                    out->dst_kind = OPKIND_IMMEDIATE;
                    out->dst_value = read_w(mem, pc + size);
                    size += 2;
                } else {
                    out->dst_kind = OPKIND_AUTOINC;
                }
                break;
            }
        }

        out->size = size;
        return size;
    }

    /* Jump instructions (opcodes 2-3) */
    if (op == 2 || op == 3) {
        out->itype = ITYPE_JUMP;
        out->jmp_condition = (instr >> 10) & 0x7;
        int offset = instr & 0x3ff;
        if (offset & 0x200) offset |= 0xfc00;
        out->jmp_offset = (int16_t)(offset * 2);
        out->size = 2;
        return 2;
    }

    /* Two-operand instructions (opcodes 4-F) */
    out->itype = ITYPE_TWO_OP;
    out->operation = op;
    out->byte_mode = (instr & 0x0040) ? 1 : 0;

    int src_register = (instr >> 8) & 0xf;
    int dst_register = instr & 0xf;
    int ad = (instr >> 7) & 0x1;
    int as = (instr >> 4) & 0x3;

    /* Resolve source operand */
    if (src_register == MSP430_CG2) {
        out->src_kind = OPKIND_CG;
        out->src_reg = src_register;
        out->src_value = cg2_vals[as];
    } else if (src_register == MSP430_CG1 && as >= 2) {
        out->src_kind = OPKIND_CG;
        out->src_reg = src_register;
        out->src_value = cg1_vals[as];
    } else {
        out->src_reg = src_register;
        switch (as) {
        case AM_REG:
            out->src_kind = OPKIND_REG;
            break;
        case AM_INDEX:
            if (src_register == MSP430_SR) {
                out->src_kind = OPKIND_ABSOLUTE;
            } else {
                out->src_kind = OPKIND_INDEXED;
            }
            out->src_value = sign_ext_16(read_w(mem, pc + size));
            size += 2;
            break;
        case AM_IND_REG:
            out->src_kind = OPKIND_INDIRECT;
            break;
        case AM_IND_AUTOINC:
            if (src_register == MSP430_PC) {
                out->src_kind = OPKIND_IMMEDIATE;
                out->src_value = read_w(mem, pc + size);
                size += 2;
            } else {
                out->src_kind = OPKIND_AUTOINC;
            }
            break;
        }
    }

    /* Resolve destination operand */
    out->dst_reg = dst_register;
    if (ad == 0) {
        out->dst_kind = OPKIND_REG;
    } else {
        if (dst_register == MSP430_SR) {
            out->dst_kind = OPKIND_ABSOLUTE;
        } else {
            out->dst_kind = OPKIND_INDEXED;
        }
        out->dst_value = sign_ext_16(read_w(mem, pc + size));
        size += 2;
    }

    out->size = size;
    return size;
}

/*
 * Decode a basic block.
 * Terminates at: jump, CALL, RETI, write to PC as dst, MSP430X,
 * or MAX_BLOCK_SIZE instructions.
 */
int msp430_decode_block(const uint8_t *mem, uint32_t pc, uint32_t max_mem,
                        basic_block_t *block) {
    block->start_pc = pc;
    block->length = 0;

    while (block->length < MAX_BLOCK_SIZE && pc + 1 < max_mem) {
        int idx = block->length;
        decoded_insn_t *di = &block->insns[idx];
        block->pc_addrs[idx] = pc;

        int sz = msp430_decode(mem, pc, max_mem, di);
        pc += sz;
        block->length++;

        /* Block terminates after this instruction if: */
        if (di->itype == ITYPE_JUMP) break;

        if (di->itype == ITYPE_SINGLE_OP) {
            int single_op = di->operation << 7;
            if (single_op == OP_CALL || single_op == OP_RETI) break;
        }

        if (di->itype == ITYPE_MSP430X) break;

        /* Two-op writing to PC or SR terminates block */
        if (di->itype == ITYPE_TWO_OP && di->dst_kind == OPKIND_REG) {
            if (di->dst_reg == MSP430_PC) break;
            /* Writing to SR can change interrupts/LPM */
            if (di->dst_reg == MSP430_SR && di->operation != OP_BIT &&
                di->operation != OP_CMP) break;
        }
    }

    block->end_pc = pc;
    return block->length;
}
