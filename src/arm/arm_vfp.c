/*
 * ARM Cortex-M4F single-precision VFP (FPv4-SP-D16) — interpreter.
 *
 * Implements the subset of FPv4-SP that Contiki-NG firmware compiled
 * for cortex-m4f actually emits: VPUSH/VPOP/VLDR/VSTR (load/store and
 * stack save), VMOV between core regs and FPU regs (ABI marshalling),
 * the IEEE 754 binary32 arithmetic ops (VADD/VSUB/VMUL/VDIV/VABS/VNEG/
 * VSQRT), comparisons (VCMP), and conversions (VCVT to/from int).
 *
 * coproc=B (double-precision encoding) is accepted only for VPUSH /
 * VPOP / VLDM / VSTM, where M4F's GCC uses it as a "push/pop a pair of
 * single-precision regs atomically" alias (s16/s17 = d8). Real
 * double-precision arithmetic on M4F is illegal and we reject it.
 *
 * Anything we don't handle returns false so the dispatcher can fault
 * loudly with PC + opcode — silent NOPs would silently corrupt
 * arithmetic results downstream.
 *
 * Encoding reference: ARMv7-M ARM A6.6 / ARMv7-A ARM A8.6.{295,353,...}.
 */
#include "arm_cpu.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

static inline float u32_to_f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }
static inline uint32_t f_to_u32(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

/* SP-reg index helpers: D bit is hw1[6] for Vd, hw1[7] for Vn (in
 * data-processing — hw1[7] is W elsewhere), hw2[5] for Vm. */
static inline int sreg_d(uint16_t hw1, uint16_t hw2) {
    return (((hw2 >> 12) & 0xF) << 1) | ((hw1 >> 6) & 1);
}
static inline int sreg_n_dp(uint16_t hw1, uint16_t hw2) {
    (void)hw2; return ((hw1 & 0xF) << 1) | ((hw1 >> 7) & 1);
}
static inline int sreg_m(uint16_t hw1, uint16_t hw2) {
    (void)hw1; return ((hw2 & 0xF) << 1) | ((hw2 >> 5) & 1);
}

bool arm_vfp_step(arm_cpu_t *cpu, uint16_t hw1, uint16_t hw2) {
    /* Coprocessor field hw2[11:8]: 0xA=SP, 0xB=DP-encoding (alias). */
    uint32_t coproc = (hw2 >> 8) & 0xF;
    if (coproc != 0xA && coproc != 0xB) return false;
    bool dp_alias = (coproc == 0xB);

    /* ---- VMOV (core ↔ single) ----
     * hw1 = 1110_1110_000_o_Vn        o = hw1[4]: 0 = Rt→Sn, 1 = Sn→Rt
     * hw2 = Rt   1010   N(0)(0)1_0000
     */
    if ((hw1 & 0xFFE0) == 0xEE00 && (hw2 & 0x007F) == 0x0010 && coproc == 0xA) {
        int o  = (hw1 >> 4) & 1;
        int n  = ((hw1 & 0xF) << 1) | ((hw2 >> 7) & 1);
        int rt = (hw2 >> 12) & 0xF;
        if (o) cpu->reg[rt] = cpu->vfp_s[n];
        else   cpu->vfp_s[n] = cpu->reg[rt];
        return true;
    }

    /* ---- VMSR FPSCR, Rt ----
     * hw1 = 1110_1110_1110_0001
     * hw2 = Rt   1010   0001_0000
     */
    if (hw1 == 0xEEE1 && (hw2 & 0x0FFF) == 0x0A10) {
        cpu->fpscr = cpu->reg[(hw2 >> 12) & 0xF];
        return true;
    }
    /* ---- VMRS Rt, FPSCR ----
     * hw1 = 1110_1110_1111_0001
     * hw2 = Rt   1010   0001_0000   (Rt=15 → APSR_nzcv ← FPSCR[31:28])
     */
    if (hw1 == 0xEEF1 && (hw2 & 0x0FFF) == 0x0A10) {
        int rt = (hw2 >> 12) & 0xF;
        if (rt == 15)
            cpu->xpsr = (cpu->xpsr & 0x0FFFFFFFu) | (cpu->fpscr & 0xF0000000u);
        else
            cpu->reg[rt] = cpu->fpscr;
        return true;
    }

    /* ---- VPUSH (T1) ----
     * hw1 = 1110_1101_0D10_1101 = 0xED2D | (D<<6); D bit only differs
     * hw2 = Vd<3:0>:1010_or_1011:imm8
     */
    if ((hw1 & 0xFFBF) == 0xED2D) {
        uint32_t imm8 = hw2 & 0xFF;
        int sd = sreg_d(hw1, hw2);
        int regs = dp_alias ? (int)imm8 * 2 : (int)imm8;
        if (dp_alias) sd &= ~1;
        if (regs == 0 || sd + regs > 32) return false;
        uint32_t sp = cpu->reg[13];
        uint32_t newsp = sp - (uint32_t)(regs * 4);
        for (int i = 0; i < regs; i++)
            arm_write32(cpu, newsp + (uint32_t)(i * 4), cpu->vfp_s[sd + i]);
        cpu->reg[13] = newsp;
        return true;
    }

    /* ---- VPOP (T1) ----
     * hw1 = 1110_1100_1D11_1101 = 0xECBD | (D<<6)
     */
    if ((hw1 & 0xFFBF) == 0xECBD) {
        uint32_t imm8 = hw2 & 0xFF;
        int sd = sreg_d(hw1, hw2);
        int regs = dp_alias ? (int)imm8 * 2 : (int)imm8;
        if (dp_alias) sd &= ~1;
        if (regs == 0 || sd + regs > 32) return false;
        uint32_t sp = cpu->reg[13];
        for (int i = 0; i < regs; i++)
            cpu->vfp_s[sd + i] = arm_read32(cpu, sp + (uint32_t)(i * 4));
        cpu->reg[13] = sp + (uint32_t)(regs * 4);
        return true;
    }

    /* ---- VLDR.32 / VSTR.32 (immediate) ----
     * hw1 = 1110_1101_UD0L_Rn  (U=hw1[7], D=hw1[6], L=hw1[4])
     * hw2 = Vd<3:0>:1010:imm8
     * Pre-indexed, no writeback.
     */
    /* VLDR/VSTR T1: hw1 = 1110_1101_UD0L_Rn — bits 15..8 = 11101101,
     * bit 5 must be 0. Mask 0xFF20 covers high byte + bit 5. */
    if ((hw1 & 0xFF20) == 0xED00) {
        int U  = (hw1 >> 7) & 1;
        int L  = (hw1 >> 4) & 1;
        int rn = hw1 & 0xF;
        uint32_t imm32 = (uint32_t)(hw2 & 0xFF) << 2;
        int sd = sreg_d(hw1, hw2);
        uint32_t base = (rn == 15) ? ((cpu->reg[15] & ~3u) - 4) : cpu->reg[rn];
        uint32_t addr = U ? base + imm32 : base - imm32;
        if (L) cpu->vfp_s[sd] = arm_read32(cpu, addr);
        else   arm_write32(cpu, addr, cpu->vfp_s[sd]);
        return true;
    }

    /* ---- VLDM / VSTM (general; same shape as VPUSH/VPOP, Rn ≠ SP) ----
     * IA: hw1 = 1110_1100_1D_W_L_Rn       (P=0, U=1)
     * DB: hw1 = 1110_1101_0D_W_L_Rn       (P=1, U=0)
     */
    /* VLDM / VSTM IA: hw1 = 1110_1100_1D_W_L_Rn (high byte 0xEC, bit 8 = 1)
     * VLDM / VSTM DB: hw1 = 1110_1101_0D_W_L_Rn (high byte 0xED, bit 8 = 0)
     * Bit 5 (the W bit) varies. */
    if (((hw1 & 0xFF00) == 0xEC00 || (hw1 & 0xFF00) == 0xED00) &&
        (hw1 & 0xF) != 13) {
        /* Already handled VPUSH/VPOP/VLDR/VSTR specifically above; this
         * is a catchall for the multi-reg form on a non-SP base. Use
         * full PUWL bits to decide direction. */
        int P = (hw1 >> 8) & 1;   /* hw1[8] in the LDC/STC space */
        int U = (hw1 >> 7) & 1;
        int W = (hw1 >> 5) & 1;
        int L = (hw1 >> 4) & 1;
        int rn = hw1 & 0xF;
        uint8_t imm8 = hw2 & 0xFF;
        int sd = sreg_d(hw1, hw2);
        int regs = dp_alias ? (int)imm8 * 2 : (int)imm8;
        if (dp_alias) sd &= ~1;
        if (regs == 0 || sd + regs > 32) return false;
        uint32_t base = cpu->reg[rn];
        uint32_t addr;
        if (P && !U) {            /* DB */
            addr = base - (uint32_t)(regs * 4);
            if (W) cpu->reg[rn] = addr;
        } else if (!P && U) {     /* IA */
            addr = base;
            if (W) cpu->reg[rn] = base + (uint32_t)(regs * 4);
        } else {
            return false;         /* Other PU combos not encoded for VFP LD/ST-multiple */
        }
        for (int i = 0; i < regs; i++) {
            if (L) cpu->vfp_s[sd + i] = arm_read32(cpu, addr + (uint32_t)(i * 4));
            else   arm_write32(cpu, addr + (uint32_t)(i * 4), cpu->vfp_s[sd + i]);
        }
        return true;
    }

    /* ---- Single-precision data-processing ----
     * Form: hw1 = 1110_1110_opc1_Vn      hw2 = Vd:1010:N_opc2_M_0_Vm
     * opc1 = hw1[7:4] (4 bits), opc2 = hw2[7:6], opc3 in hw2[6] for some.
     */
    if ((hw1 & 0xFF00) == 0xEE00 && (hw2 & 0x0F00) == 0x0A00) {
        int sd = sreg_d(hw1, hw2);
        int sn = sreg_n_dp(hw1, hw2);
        int sm = sreg_m(hw1, hw2);
        int opc1 = (hw1 >> 4) & 0xF;
        int opc3 = (hw2 >> 6) & 0x3;
        float vn = u32_to_f(cpu->vfp_s[sn]);
        float vm = u32_to_f(cpu->vfp_s[sm]);
        float vd = u32_to_f(cpu->vfp_s[sd]);
        bool tbit = (hw1 >> 6) & 1;
        (void)tbit;

        switch (opc1 & 0xB) {
            case 0x0:    /* VMLA / VMLS */
                if (opc3 & 1) cpu->vfp_s[sd] = f_to_u32(vd - vn * vm);
                else          cpu->vfp_s[sd] = f_to_u32(vd + vn * vm);
                return true;
            case 0x2:    /* VMUL / VNMUL */
                if (opc3 & 1) cpu->vfp_s[sd] = f_to_u32(-(vn * vm));
                else          cpu->vfp_s[sd] = f_to_u32(vn * vm);
                return true;
            case 0x3:    /* VADD / VSUB */
                if (opc3 & 1) cpu->vfp_s[sd] = f_to_u32(vn - vm);
                else          cpu->vfp_s[sd] = f_to_u32(vn + vm);
                return true;
            case 0x8:    /* VDIV */
                cpu->vfp_s[sd] = f_to_u32(vn / vm);
                return true;
            case 0xB: {  /* misc (VABS/VNEG/VSQRT/VMOV/VCMP/VCVT/...) */
                int op = (hw2 >> 16) & 0; /* placeholder, real decode below */
                (void)op;
                int opc2 = (hw1 & 0xF);  /* hw1[3:0] when opc1 == 0xB */
                /* hw1[3:0] semantics for the 0xB family: VMOV/VABS/VNEG/
                 * VSQRT/VCVT/VCMP. Use hw2 opc3 to discriminate. */
                switch (opc2) {
                    case 0x0:
                        if (opc3 == 1) { cpu->vfp_s[sd] = cpu->vfp_s[sm]; return true; }   /* VMOV */
                        if (opc3 == 3) { cpu->vfp_s[sd] = f_to_u32(fabsf(vm)); return true; } /* VABS */
                        break;
                    case 0x1:
                        if (opc3 == 1) { cpu->vfp_s[sd] = f_to_u32(-vm); return true; }       /* VNEG */
                        if (opc3 == 3) { cpu->vfp_s[sd] = f_to_u32(sqrtf(vm)); return true; } /* VSQRT */
                        break;
                    case 0x4: case 0x5: {  /* VCMP / VCMPE */
                        float a = vd;
                        float b = (opc2 == 0x5) ? 0.0f : vm;
                        uint32_t flags;
                        if (isnan(a) || isnan(b))      flags = 0x30000000u;
                        else if (a == b)               flags = 0x60000000u;
                        else if (a < b)                flags = 0x80000000u;
                        else                           flags = 0x20000000u;
                        cpu->fpscr = (cpu->fpscr & 0x0FFFFFFFu) | flags;
                        return true;
                    }
                    case 0x8: {  /* VCVT.F32.{S,U}32 — int → float */
                        int unsigned_op = (hw2 >> 7) & 1;
                        if (unsigned_op) cpu->vfp_s[sd] = f_to_u32((float)cpu->vfp_s[sm]);
                        else             cpu->vfp_s[sd] = f_to_u32((float)(int32_t)cpu->vfp_s[sm]);
                        return true;
                    }
                    case 0xC: case 0xD: {  /* VCVT.{U,S}32.F32 — float → int (round to zero) */
                        int signed_op = (opc2 == 0xD);
                        if (signed_op) {
                            int32_t v = (int32_t)vm;
                            cpu->vfp_s[sd] = (uint32_t)v;
                        } else {
                            uint32_t v = (vm < 0.0f) ? 0 : (uint32_t)vm;
                            cpu->vfp_s[sd] = v;
                        }
                        return true;
                    }
                }
                break;
            }
        }
    }

    return false;
}
