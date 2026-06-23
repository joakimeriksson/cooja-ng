/*
 * RV32E (rv32e_zicsr_zifencei) interpreter — see include/riscv/riscv_cpu.h.
 *
 * All memory + IO goes through the shared M33 bus (arm_read / arm_write
 * helpers), so the FLPR and the M33 see one address space. Soft mul/div in
 * the FLPR firmware are ordinary libgcc base-RV32I instructions — no M
 * extension needed here.
 */
#include "riscv_cpu.h"
#include "arm_cpu.h"
#include <string.h>

/* Machine CSR numbers (the ones nrf-vpr's startup-stubs.c + traps touch). */
#define CSR_MSTATUS   0x300
#define CSR_MISA      0x301
#define CSR_MIE       0x304
#define CSR_MTVEC     0x305
#define CSR_MSCRATCH  0x340
#define CSR_MEPC      0x341
#define CSR_MCAUSE    0x342
#define CSR_MTVAL     0x343
#define CSR_MIP       0x344
#define CSR_MHARTID   0xF14

/* Trap causes (mcause) used here. */
#define CAUSE_INSN_MISALIGNED  0
#define CAUSE_ILLEGAL_INSN     2
#define CAUSE_BREAKPOINT       3
#define CAUSE_ECALL_M          11

void riscv_cpu_init(riscv_cpu_t *rv, struct arm_cpu *bus, uint32_t initpc) {
    memset(rv, 0, sizeof(*rv));
    rv->bus = bus;
    rv->pc  = initpc;
}

/* --- memory: borrow the host bus so SRAM + IO are shared with the M33 --- */
static inline uint32_t ld32(riscv_cpu_t *rv, uint32_t a) { return arm_read32(rv->bus, a); }
static inline uint16_t ld16(riscv_cpu_t *rv, uint32_t a) { return arm_read16(rv->bus, a); }
static inline uint8_t  ld8 (riscv_cpu_t *rv, uint32_t a) { return arm_read8 (rv->bus, a); }
static inline void st32(riscv_cpu_t *rv, uint32_t a, uint32_t v) { arm_write32(rv->bus, a, v); }
static inline void st16(riscv_cpu_t *rv, uint32_t a, uint16_t v) { arm_write16(rv->bus, a, v); }
static inline void st8 (riscv_cpu_t *rv, uint32_t a, uint8_t  v) { arm_write8 (rv->bus, a, v); }

static uint32_t csr_read(riscv_cpu_t *rv, uint32_t csr) {
    switch (csr) {
        case CSR_MSTATUS:  return rv->mstatus;
        case CSR_MISA:     return 0x40000010u; /* RV32 (MXL=1) + 'E' (bit 4) */
        case CSR_MIE:      return rv->mie;
        case CSR_MTVEC:    return rv->mtvec;
        case CSR_MSCRATCH: return rv->mscratch;
        case CSR_MEPC:     return rv->mepc;
        case CSR_MCAUSE:   return rv->mcause;
        case CSR_MTVAL:    return rv->mtval;
        case CSR_MIP:      return rv->mip;
        case CSR_MHARTID:  return 0;
        default:           return 0;
    }
}

static void csr_write(riscv_cpu_t *rv, uint32_t csr, uint32_t v) {
    switch (csr) {
        case CSR_MSTATUS:  rv->mstatus  = v; break;
        case CSR_MIE:      rv->mie      = v; break;
        case CSR_MTVEC:    rv->mtvec    = v; break;
        case CSR_MSCRATCH: rv->mscratch = v; break;
        case CSR_MEPC:     rv->mepc     = v; break;
        case CSR_MCAUSE:   rv->mcause   = v; break;
        case CSR_MTVAL:    rv->mtval    = v; break;
        case CSR_MIP:      rv->mip      = v; break;
        default: break; /* read-only / unimplemented: ignore */
    }
}

static void take_trap(riscv_cpu_t *rv, uint32_t cause, uint32_t tval, uint32_t epc) {
    rv->mepc   = epc;
    rv->mcause = cause;
    rv->mtval  = tval;
    /* mstatus: MPIE <- MIE, MIE <- 0, MPP <- M(3). */
    uint32_t st  = rv->mstatus;
    uint32_t mie = (st >> 3) & 1u;
    st = (st & ~(1u << 7)) | (mie << 7);
    st &= ~(1u << 3);
    st |= (3u << 11);
    rv->mstatus = st;
    rv->pc      = rv->mtvec & ~3u;  /* direct mode */
    rv->trap_pc = epc;
}

int riscv_step(riscv_cpu_t *rv, int n) {
    int i;
    for (i = 0; i < n; i++) {
        if (rv->halted) break;

        uint32_t pc = rv->pc;
        if (pc & 1u) { take_trap(rv, CAUSE_INSN_MISALIGNED, pc, pc); continue; }

        uint32_t insn = ld32(rv, pc);
        rv->cycles++;


        uint32_t opcode = insn & 0x7fu;
        uint32_t rd     = (insn >> 7)  & 0x1fu;
        uint32_t rs1    = (insn >> 15) & 0x1fu;
        uint32_t rs2    = (insn >> 20) & 0x1fu;
        uint32_t funct3 = (insn >> 12) & 7u;
        uint32_t funct7 = (insn >> 25) & 0x7fu;
        uint32_t next   = pc + 4u;

        #define R(idx)     (rv->x[(idx)])
        #define SET(d, v)  do { if (d) rv->x[(d)] = (uint32_t)(v); } while (0)

        switch (opcode) {
        case 0x37: /* LUI   */ SET(rd, insn & 0xfffff000u); break;
        case 0x17: /* AUIPC */ SET(rd, pc + (insn & 0xfffff000u)); break;

        case 0x6f: { /* JAL */
            int32_t imm = (int32_t)(
                (((insn >> 31) & 1u) << 20) |
                (((insn >> 12) & 0xffu) << 12) |
                (((insn >> 20) & 1u) << 11) |
                (((insn >> 21) & 0x3ffu) << 1));
            imm = (imm << 11) >> 11;            /* sign-extend 21-bit */
            SET(rd, next);
            next = pc + (uint32_t)imm;
            break;
        }
        case 0x67: { /* JALR */
            int32_t imm = (int32_t)insn >> 20;
            uint32_t t = (R(rs1) + (uint32_t)imm) & ~1u;
            SET(rd, next);
            next = t;
            break;
        }
        case 0x63: { /* BRANCH */
            int32_t imm = (int32_t)(
                (((insn >> 31) & 1u) << 12) |
                (((insn >> 7)  & 1u) << 11) |
                (((insn >> 25) & 0x3fu) << 5) |
                (((insn >> 8)  & 0xfu) << 1));
            imm = (imm << 19) >> 19;            /* sign-extend 13-bit */
            uint32_t a = R(rs1), b = R(rs2);
            int take = 0;
            switch (funct3) {
                case 0: take = (a == b); break;                       /* BEQ  */
                case 1: take = (a != b); break;                       /* BNE  */
                case 4: take = ((int32_t)a <  (int32_t)b); break;     /* BLT  */
                case 5: take = ((int32_t)a >= (int32_t)b); break;     /* BGE  */
                case 6: take = (a <  b); break;                       /* BLTU */
                case 7: take = (a >= b); break;                       /* BGEU */
                default: take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc); goto cont;
            }
            if (take) next = pc + (uint32_t)imm;
            break;
        }
        case 0x03: { /* LOAD */
            int32_t imm = (int32_t)insn >> 20;
            uint32_t a = R(rs1) + (uint32_t)imm;
            uint32_t v;
            switch (funct3) {
                case 0: v = (uint32_t)(int32_t)(int8_t)ld8(rv, a); break;   /* LB  */
                case 1: v = (uint32_t)(int32_t)(int16_t)ld16(rv, a); break; /* LH  */
                case 2: v = ld32(rv, a); break;                            /* LW  */
                case 4: v = ld8(rv, a); break;                             /* LBU */
                case 5: v = ld16(rv, a); break;                            /* LHU */
                default: take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc); goto cont;
            }
            SET(rd, v);
            break;
        }
        case 0x23: { /* STORE */
            int32_t imm = (((int32_t)insn >> 25) << 5) | (int32_t)((insn >> 7) & 0x1fu);
            uint32_t a = R(rs1) + (uint32_t)imm;
            switch (funct3) {
                case 0: st8 (rv, a, (uint8_t)R(rs2)); break;   /* SB */
                case 1: st16(rv, a, (uint16_t)R(rs2)); break;  /* SH */
                case 2: st32(rv, a, R(rs2)); break;            /* SW */
                default: take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc); goto cont;
            }
            break;
        }
        case 0x13: { /* OP-IMM */
            int32_t  imm = (int32_t)insn >> 20;
            uint32_t a = R(rs1), v;
            switch (funct3) {
                case 0: v = a + (uint32_t)imm; break;                  /* ADDI  */
                case 2: v = ((int32_t)a < imm); break;                 /* SLTI  */
                case 3: v = (a < (uint32_t)imm); break;                /* SLTIU */
                case 4: v = a ^ (uint32_t)imm; break;                  /* XORI  */
                case 6: v = a | (uint32_t)imm; break;                  /* ORI   */
                case 7: v = a & (uint32_t)imm; break;                  /* ANDI  */
                case 1: v = a << (imm & 31); break;                    /* SLLI  */
                case 5: v = (funct7 & 0x20)                            /* SRAI/SRLI */
                            ? (uint32_t)((int32_t)a >> (imm & 31))
                            : (a >> (imm & 31));
                        break;
                default: take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc); goto cont;
            }
            SET(rd, v);
            break;
        }
        case 0x33: { /* OP (register-register) */
            if (funct7 == 0x01) { /* M extension is absent on rv32e here */
                take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc); goto cont;
            }
            uint32_t a = R(rs1), b = R(rs2), v;
            switch (funct3) {
                case 0: v = (funct7 & 0x20) ? a - b : a + b; break;    /* SUB/ADD */
                case 1: v = a << (b & 31); break;                      /* SLL  */
                case 2: v = ((int32_t)a < (int32_t)b); break;          /* SLT  */
                case 3: v = (a < b); break;                            /* SLTU */
                case 4: v = a ^ b; break;                              /* XOR  */
                case 5: v = (funct7 & 0x20)                            /* SRA/SRL */
                            ? (uint32_t)((int32_t)a >> (b & 31))
                            : (a >> (b & 31));
                        break;
                case 6: v = a | b; break;                              /* OR   */
                case 7: v = a & b; break;                              /* AND  */
                default: take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc); goto cont;
            }
            SET(rd, v);
            break;
        }
        case 0x0f: /* FENCE / FENCE.I (Zifencei) — no reordering to model */
            break;

        case 0x73: { /* SYSTEM */
            if (funct3 == 0) {
                uint32_t imm12 = insn >> 20;
                if (imm12 == 0x000) {            /* ECALL  */
                    take_trap(rv, CAUSE_ECALL_M, 0, pc); goto cont;
                } else if (imm12 == 0x001) {     /* EBREAK */
                    take_trap(rv, CAUSE_BREAKPOINT, 0, pc); goto cont;
                } else if (imm12 == 0x302) {     /* MRET   */
                    uint32_t st = rv->mstatus;
                    uint32_t mpie = (st >> 7) & 1u;
                    st = (st & ~(1u << 3)) | (mpie << 3); /* MIE <- MPIE */
                    st |= (1u << 7);                       /* MPIE <- 1   */
                    rv->mstatus = st;
                    next = rv->mepc;
                }
                /* WFI (0x105) and other privileged hints: treat as nop. */
            } else {
                /* Zicsr: CSRRW/S/C and immediate forms. */
                uint32_t csr = insn >> 20;
                uint32_t old = csr_read(rv, csr);
                uint32_t src = (funct3 & 4) ? rs1 /* zimm */ : R(rs1);
                uint32_t neu = old;
                switch (funct3 & 3) {
                    case 1: neu = src; break;        /* CSRRW(I) */
                    case 2: neu = old | src; break;  /* CSRRS(I) */
                    case 3: neu = old & ~src; break; /* CSRRC(I) */
                    default: break;
                }
                /* CSRRW always writes; CSRRS/C skip the write when rs1/zimm==0. */
                if ((funct3 & 3) == 1 || rs1 != 0)
                    csr_write(rv, csr, neu);
                SET(rd, old);
            }
            break;
        }

        default:
            take_trap(rv, CAUSE_ILLEGAL_INSN, insn, pc);
            goto cont;
        }

        rv->pc = next;
    cont:;
        #undef R
        #undef SET
    }
    return i;
}

void riscv_step_until(riscv_cpu_t *rv, int64_t target_cycle) {
    while (rv->cycles < target_cycle && !rv->halted) {
        if (riscv_step(rv, 1024) == 0) break;
    }
}
