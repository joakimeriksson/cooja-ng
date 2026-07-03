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

/* ---- RV32C: expand a 16-bit compressed insn to its 32-bit equivalent ----
 * The FLPR blob is built -march=rv32emc, so gcc emits compressed ops. Rather
 * than a second decoder, decompress each to its canonical 32-bit form and run
 * the existing path. Returns 0 (an illegal encoding) for reserved/unsupported
 * forms. 3-bit reg fields map to x8..x15 (fine for RV32E). */
#define CB(x,b)        (((uint32_t)(x) >> (b)) & 1u)
#define CBITS(x,hi,lo) (((uint32_t)(x) >> (lo)) & ((1u << ((hi)-(lo)+1)) - 1u))
static inline uint32_t rvc_sext(uint32_t v, int bits) { uint32_t m = 1u << (bits-1); return (v ^ m) - m; }
static inline uint32_t enc_i(uint32_t op,uint32_t rd,uint32_t f3,uint32_t rs1,uint32_t imm){
    return op | (rd<<7) | (f3<<12) | (rs1<<15) | ((imm & 0xfffu)<<20); }
static inline uint32_t enc_r(uint32_t op,uint32_t rd,uint32_t f3,uint32_t rs1,uint32_t rs2,uint32_t f7){
    return op | (rd<<7) | (f3<<12) | (rs1<<15) | (rs2<<20) | (f7<<25); }
static inline uint32_t enc_s(uint32_t op,uint32_t f3,uint32_t rs1,uint32_t rs2,uint32_t imm){
    return op | ((imm&0x1fu)<<7) | (f3<<12) | (rs1<<15) | (rs2<<20) | (((imm>>5)&0x7fu)<<25); }
static inline uint32_t enc_b(uint32_t op,uint32_t f3,uint32_t rs1,uint32_t rs2,uint32_t imm){
    return op | (((imm>>11)&1u)<<7)|(((imm>>1)&0xfu)<<8)|(f3<<12)|(rs1<<15)|(rs2<<20)
             |(((imm>>5)&0x3fu)<<25)|(((imm>>12)&1u)<<31); }
static inline uint32_t enc_j(uint32_t op,uint32_t rd,uint32_t imm){
    return op | (rd<<7) | (((imm>>12)&0xffu)<<12)|(((imm>>11)&1u)<<20)
             |(((imm>>1)&0x3ffu)<<21)|(((imm>>20)&1u)<<31); }

static uint32_t rvc_expand(uint16_t ci) {
    uint32_t c = ci;
    uint32_t f3 = CBITS(c,15,13);
    uint32_t rdp = 8u + CBITS(c,9,7), rs2p = 8u + CBITS(c,4,2);
    uint32_t rd = CBITS(c,11,7), rs2 = CBITS(c,6,2);
    /* CJ / CB format immediates */
    uint32_t immj = (CB(c,12)<<11)|(CB(c,11)<<4)|(CBITS(c,10,9)<<8)|(CB(c,8)<<10)
                  |(CB(c,7)<<6)|(CB(c,6)<<7)|(CBITS(c,5,3)<<1)|(CB(c,2)<<5);
    uint32_t immb = (CB(c,12)<<8)|(CBITS(c,11,10)<<3)|(CBITS(c,6,5)<<6)|(CBITS(c,4,3)<<1)|(CB(c,2)<<5);
    uint32_t sh = (CB(c,12)<<5)|CBITS(c,6,2);
    switch (c & 3u) {
    case 0: switch (f3) {
        case 0: { uint32_t imm=(CBITS(c,10,7)<<6)|(CBITS(c,12,11)<<4)|(CB(c,5)<<3)|(CB(c,6)<<2);
                  return imm ? enc_i(0x13, 8u+CBITS(c,4,2), 0, 2, imm) : 0; } /* C.ADDI4SPN */
        case 2: { uint32_t off=(CB(c,5)<<6)|(CBITS(c,12,10)<<3)|(CB(c,6)<<2);
                  return enc_i(0x03, 8u+CBITS(c,4,2), 2, rdp, off); }          /* C.LW */
        case 6: { uint32_t off=(CB(c,5)<<6)|(CBITS(c,12,10)<<3)|(CB(c,6)<<2);
                  return enc_s(0x23, 2, rdp, rs2p, off); }                     /* C.SW */
        default: return 0; }
    case 1: switch (f3) {
        case 0: return enc_i(0x13, rd, 0, rd, rvc_sext(sh,6));                 /* C.ADDI/NOP */
        case 1: return enc_j(0x6f, 1, rvc_sext(immj,12));                      /* C.JAL (RV32) */
        case 2: return enc_i(0x13, rd, 0, 0, rvc_sext(sh,6));                  /* C.LI */
        case 3:
            if (rd==2) { uint32_t imm=(CB(c,12)<<9)|(CBITS(c,4,3)<<7)|(CB(c,5)<<6)|(CB(c,2)<<5)|(CB(c,6)<<4);
                         return imm ? enc_i(0x13,2,0,2,rvc_sext(imm,10)) : 0; } /* C.ADDI16SP */
            else       { uint32_t nz=(CB(c,12)<<5)|CBITS(c,6,2);
                         return nz ? (0x37u|(rd<<7)|((rvc_sext(nz,6)<<12)&0xfffff000u)) : 0; } /* C.LUI */
        case 4: { uint32_t f2=CBITS(c,11,10);
            if (f2==0) return enc_i(0x13, rdp, 5, rdp, sh & 0x1fu);            /* C.SRLI */
            if (f2==1) return enc_i(0x13, rdp, 5, rdp, 0x400u|(sh & 0x1fu));   /* C.SRAI */
            if (f2==2) return enc_i(0x13, rdp, 7, rdp, rvc_sext(sh,6));        /* C.ANDI */
            if (CB(c,12)) return 0;                                            /* SUBW/ADDW: RV64 */
            switch (CBITS(c,6,5)) {
            case 0: return enc_r(0x33, rdp, 0, rdp, rs2p, 0x20);               /* C.SUB */
            case 1: return enc_r(0x33, rdp, 4, rdp, rs2p, 0x00);               /* C.XOR */
            case 2: return enc_r(0x33, rdp, 6, rdp, rs2p, 0x00);               /* C.OR  */
            default:return enc_r(0x33, rdp, 7, rdp, rs2p, 0x00); }             /* C.AND */
        }
        case 5: return enc_j(0x6f, 0, rvc_sext(immj,12));                     /* C.J */
        case 6: return enc_b(0x63, 0, rdp, 0, rvc_sext(immb,9));              /* C.BEQZ */
        default:return enc_b(0x63, 1, rdp, 0, rvc_sext(immb,9)); }            /* C.BNEZ */
    case 2: switch (f3) {
        case 0: return enc_i(0x13, rd, 1, rd, sh & 0x1fu);                    /* C.SLLI */
        case 2: { uint32_t off=(CBITS(c,3,2)<<6)|(CB(c,12)<<5)|(CBITS(c,6,4)<<2);
                  return enc_i(0x03, rd, 2, 2, off); }                        /* C.LWSP */
        case 4:
            if (!CB(c,12)) return rs2 ? enc_r(0x33,rd,0,0,rs2,0)              /* C.MV */
                                      : enc_i(0x67,0,0,rd,0);                 /* C.JR */
            if (rd==0 && rs2==0) return enc_i(0x73,0,0,0,1);                  /* C.EBREAK */
            return rs2 ? enc_r(0x33,rd,0,rd,rs2,0)                            /* C.ADD */
                       : enc_i(0x67,1,0,rd,0);                               /* C.JALR */
        case 6: { uint32_t off=(CBITS(c,8,7)<<6)|(CBITS(c,12,9)<<2);
                  return enc_s(0x23, 2, 2, rs2, off); }                       /* C.SWSP */
        default: return 0; }
    }
    return 0;
}

/* Machine-mode interrupt: take the highest-priority enabled+pending IRQ,
 * entering via mtvec (direct mode, which the nrf-vpr firmware uses). Also wakes
 * the hart from WFI (rv->halted). Standard RISC-V trap entry: MPIE<-MIE, MIE<-0,
 * MPP<-M, mepc<-pc, mcause<-(interrupt|cause). Returns 1 if an IRQ was taken. */
#define RV_MSTATUS_MIE  (1u << 3)
static int riscv_take_pending_irq(riscv_cpu_t *rv) {
    uint32_t irq = rv->mip & rv->mie;
    if (!irq || !(rv->mstatus & RV_MSTATUS_MIE)) return 0;
    uint32_t cause = (irq & (1u << 11)) ? 11u    /* machine external */
                   : (irq & (1u << 7))  ? 7u     /* machine timer    */
                                        : 3u;    /* machine software */
    rv->halted = 0;
    rv->mepc   = rv->pc;
    rv->mcause = 0x80000000u | cause;
    rv->mtval  = 0;
    uint32_t st  = rv->mstatus;
    uint32_t mie = (st >> 3) & 1u;
    st = (st & ~(1u << 7)) | (mie << 7);   /* MPIE <- MIE */
    st = (st & ~(1u << 3));                 /* MIE  <- 0   */
    st |= (3u << 11);                        /* MPP  <- M   */
    rv->mstatus = st;
    rv->pc      = rv->mtvec & ~3u;           /* direct mode */
    rv->trap_pc = rv->mepc;
    return 1;
}

int riscv_step(riscv_cpu_t *rv, int n) {
    int i;
    for (i = 0; i < n; i++) {
        if (riscv_take_pending_irq(rv)) continue; /* also un-halts on wake */
        if (rv->halted) break;

        uint32_t pc = rv->pc;
        if (pc & 1u) { take_trap(rv, CAUSE_INSN_MISALIGNED, pc, pc); continue; }

        /* Fetch: a 16-bit compressed op (low 2 bits != 0b11) expands to its
         * 32-bit form; otherwise a full 32-bit word. */
        uint32_t insn = ld16(rv, pc);
        uint32_t next;
        if ((insn & 3u) == 3u) {
            /* 32-bit op: with the C extension it may be only 2-byte aligned, so
             * read the high half separately — a single ld32 would 4-byte-align
             * and fetch the wrong word. */
            insn |= (uint32_t)ld16(rv, pc + 2u) << 16;
            next = pc + 4u;
        } else {
            uint32_t comp = insn;
            insn = rvc_expand((uint16_t)comp);
            if (insn == 0u) { take_trap(rv, CAUSE_ILLEGAL_INSN, comp, pc); continue; }
            next = pc + 2u;
        }
        rv->cycles++;

        uint32_t opcode = insn & 0x7fu;
        uint32_t rd     = (insn >> 7)  & 0x1fu;
        uint32_t rs1    = (insn >> 15) & 0x1fu;
        uint32_t rs2    = (insn >> 20) & 0x1fu;
        uint32_t funct3 = (insn >> 12) & 7u;
        uint32_t funct7 = (insn >> 25) & 0x7fu;

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
            uint32_t a = R(rs1), b = R(rs2), v;
            if (funct7 == 0x01) { /* RV32M: mul / div / rem */
                switch (funct3) {
                case 0: v = (uint32_t)((int32_t)a * (int32_t)b); break;                         /* MUL    */
                case 1: v = (uint32_t)(((int64_t)(int32_t)a * (int64_t)(int32_t)b) >> 32); break;/* MULH   */
                case 2: v = (uint32_t)(((int64_t)(int32_t)a * (int64_t)b) >> 32); break;         /* MULHSU */
                case 3: v = (uint32_t)(((uint64_t)a * (uint64_t)b) >> 32); break;                /* MULHU  */
                case 4: { int32_t x=(int32_t)a,y=(int32_t)b;                                     /* DIV    */
                          v = (y==0)?0xffffffffu:(x==(int32_t)0x80000000u&&y==-1)?(uint32_t)x:(uint32_t)(x/y); break; }
                case 5: v = (b==0)?0xffffffffu:a/b; break;                                       /* DIVU   */
                case 6: { int32_t x=(int32_t)a,y=(int32_t)b;                                     /* REM    */
                          v = (y==0)?a:(x==(int32_t)0x80000000u&&y==-1)?0u:(uint32_t)(x%y); break; }
                default:v = (b==0)?a:a%b; break;                                                 /* REMU   */
                }
                SET(rd, v);
                break;
            }
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
                else if (imm12 == 0x105) {      /* WFI: idle until an IRQ    */
                    /* Retire wfi (pc already advanced to next), then halt; the
                     * top-of-loop IRQ check un-halts and vectors when an enabled
                     * interrupt becomes pending. Skips burning the co-step budget
                     * while idle — the whole point of the fix. */
                    rv->halted = 1;
                }
                /* other privileged hints: treat as nop. */
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
